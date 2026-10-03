#include "daisysp.h"
#include "daisy_pod.h"
#include <math.h>

// ============================================================================
// MultiEffect + Dub Siren + Auto-Pan
//
// Chris's effects box: five modes cycled by the buttons (REV/DEL/CRU/FIL/PAN),
// plus the dub siren grafted in as a SIXTH mode that lives OUTSIDE the
// rotation entirely.
//
//   * Effects mode  : five effects, cycled by the buttons.
//   * Siren mode    : the dub siren takes over. Buttons become triggers.
//
//   Toggle between the two worlds: HOLD BUTTON 1 + BUTTON 2 together for ~1
//   second. Both LEDs go white while you hold; release into the new mode's
//   colors.
//
// NOTE: effect switching fires on button RELEASE (FallingEdge) instead of
// press, so the both-buttons hold can't step an effect mid-combo.
//
// PAN mode notes:
//   Knob 1   = Rate (ignored while synced - see below)
//   Knob 2   = Depth
//   Encoder turn        = Shape (smooth sine -> sharp near-square sweep)
//   Encoder short press = cycle Waveform (sine -> triangle -> square)
//   Encoder long press  = toggle sync: free-running Hz <-> locked to the
//                         delay's time (one full L-R sweep per echo repeat,
//                         using whatever currentDelay is set to - tap it in
//                         from DEL mode, or dial it with DEL's knob 1)
//   LEDs                = live L/R position (red), not static like the
//                         other modes - a visual pan meter
//
// SIMPLIFIED vs. a full Ableton Auto Pan: no Random/S&H waveform (would need
// a separate noise + sample-and-hold stage), and stereo Phase/Spin between
// channels is fixed at the standard 180 degrees in the pan-law math rather
// than exposed as a control.
// ============================================================================

// Set max delay time to 2.5 sec.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)
#define REV 0
#define DEL 1
#define CRU 2
#define FIL 3
#define PAN 4

using namespace daisysp;
using namespace daisy;

static DaisyPod pod;

static ReverbSc                                  rev;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS dell;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delr;
static Tone                                      tone;
static Svf                                       svfl, svfr;
static Oscillator                                panLfo;
static Parameter deltime, cutoffParam, crushrate, filtFreqParam;
int              mode = REV;

float currentDelay, feedback, delayTarget, cutoff;

int   crushmod, crushcount;
float crushsl, crushsr, drywet;

// --- added state (effects) ---
float    sample_rate;         // file-scope so tap tempo can convert ms -> samples
float    reverbSend = 0.25f;  // how much reverb rides on the delay (0..1)
float    masterWet  = 0.5f;   // encoder-turn master wet/dry (0..1) - non-PAN modes
uint32_t lastTapTime = 0;     // ms timestamp of previous encoder tap
float    knob1Last  = -1.0f;  // for detecting knob1 movement in DEL mode
float    filtFreq;            // current filter cutoff (Hz)
int      filtType   = 0;      // 0 = LP, 1 = HP, 2 = BP
uint32_t pressStart = 0;      // ms timestamp of encoder press (short vs long)

// --- auto-pan (5th mode) ---
float panRate   = 1.0f;   // Hz, free-running
float panDepth  = 1.0f;   // 0..1
float panShape  = 0.0f;   // 0..1, 0 = pure sine, 1 = squared-off sweep
int   panWave   = 0;      // 0 = sine, 1 = triangle, 2 = square
bool  panSynced = false;  // false = free Hz (knob1), true = locked to delay time
float panAmpL   = 0.5f;   // last computed L/R amplitudes, for the LED meter
float panAmpR   = 0.5f;

static const uint8_t kPanWaves[3] = {
    Oscillator::WAVE_SIN,
    Oscillator::WAVE_TRI,
    Oscillator::WAVE_SQUARE,
};

// --- dub siren (6th mode, outside the rotation) ---
static Oscillator sirenOsc, sirenLfo;
static Svf        sirenFbFilt;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS sirenDelay;

bool  sirenMode      = false;
int   sirenPage      = 0;      // 0 = wail, 1 = echo
int   sirenWave      = 0;
int   sirenLfoWave   = 0;      // LFO shape: sine/tri/square/ramp
bool  sirenLatch     = false;
bool  sirenGate      = false;
float sirenBaseFreq  = 220.f;  // Hz
float sirenRate      = 4.f;    // Hz
float sirenDepth     = 0.f;    // Hz of pitch deviation
float sirenDelMs     = 300.f;  // ms
float sirenFeedback  = 0.5f;   // 0..1
float sirenToneHz    = 4000.f; // output LP cutoff
float sirenDelTarget, sirenDelCurrent;
float sirenAmp       = 0.f;    // click-free gate envelope
float sirenToneState = 0.f;    // output one-pole LP state
float sirenToneCoef  = 0.f;    // per-block tone coefficient
float sirenLevel     = 0.7f;   // siren volume vs. passthrough (0..1.5)
bool  sirenLevelEdit = false;  // true while adjusting siren level
int      sirenBlinks  = 0;     // white pulses left in the count sequence
bool     sirenBlinkOn = false; // LED2 currently lit for a pulse
uint32_t sirenBlinkT  = 0;     // next blink phase-change time (ms)

static const uint8_t kSirenWaves[4] = {
    Oscillator::WAVE_POLYBLEP_SQUARE,
    Oscillator::WAVE_POLYBLEP_SAW,
    Oscillator::WAVE_POLYBLEP_TRI,
    Oscillator::WAVE_SIN,
};

// LFO shapes for the wail (plain shapes — no aliasing to worry about at LFO
// rates): sine = smooth wail, tri = linear sweep, square = hard two-tone,
// ramp = rising-then-snap sweep.
static const uint8_t kLfoWaves[4] = {
    Oscillator::WAVE_SIN,
    Oscillator::WAVE_TRI,
    Oscillator::WAVE_SQUARE,
    Oscillator::WAVE_RAMP,
};

// --- both-buttons combo state ---
uint32_t comboStart   = 0;
bool     comboActive  = false; // both currently held
bool     comboFired   = false; // the 1s toggle already fired this episode
bool     comboSwallow = false; // suppress normal button jobs until both release

// Helper functions
void Controls();
void HandleModeCombo();
void SirenControls();
float GetSirenSample();

void GetReverbSample(float &outl, float &outr, float inl, float inr);
void GetDelaySample(float &outl, float &outr, float inl, float inr);
void GetCrushSample(float &outl, float &outr, float inl, float inr);
void GetFilterSample(float &outl, float &outr, float inl, float inr);
void GetPanSample(float &outl, float &outr, float inl, float inr);


void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{
    float outl, outr, inl, inr;

    Controls();

    //audio
    for(size_t i = 0; i < size; i += 2)
    {
        if(sirenMode)
        {
            // the siren rides ON TOP of the dry input: input passes straight
            // through and the voice is summed over it (a mix, not a crossfade)
            float s    = GetSirenSample();
            out[i]     = in[i]     + s;
            out[i + 1] = in[i + 1] + s;
            continue;
        }

        inl = in[i];
        inr = in[i + 1];

        switch(mode)
        {
            case REV: GetReverbSample(outl, outr, inl, inr); break;
            case DEL: GetDelaySample(outl, outr, inl, inr); break;
            case CRU: GetCrushSample(outl, outr, inl, inr); break;
            case FIL: GetFilterSample(outl, outr, inl, inr); break;
            case PAN: GetPanSample(outl, outr, inl, inr); break;
            default: outl = outr = 0;
        }

        // master wet/dry: crossfade the processed signal against untouched input
        out[i]     = (1.0f - masterWet) * inl + masterWet * outl;
        out[i + 1] = (1.0f - masterWet) * inr + masterWet * outr;
    }
}

int main(void)
{
    //Inits and sample rate
    pod.Init();
    pod.SetAudioBlockSize(4);
    sample_rate = pod.AudioSampleRate();
    rev.Init(sample_rate);
    dell.Init();
    delr.Init();
    tone.Init(sample_rate);
    svfl.Init(sample_rate);
    svfr.Init(sample_rate);

    //set parameters
    deltime.Init(pod.knob1, sample_rate * .05, MAX_DELAY, deltime.LOGARITHMIC);
    cutoffParam.Init(pod.knob1, 500, 20000, cutoffParam.LOGARITHMIC);
    crushrate.Init(pod.knob2, 1, 50, crushrate.LOGARITHMIC);
    filtFreqParam.Init(pod.knob1, 60, 16000, filtFreqParam.LOGARITHMIC);

    //reverb parameters
    rev.SetLpFreq(18000.0f);
    rev.SetFeedback(0.85f);

    //delay parameters
    currentDelay = delayTarget = sample_rate * 0.75f;
    dell.SetDelay(currentDelay);
    delr.SetDelay(currentDelay);

    //auto-pan init
    panLfo.Init(sample_rate);
    panLfo.SetWaveform(kPanWaves[0]);
    panLfo.SetFreq(panRate);
    panLfo.SetAmp(1.f);

    //dub siren init
    sirenOsc.Init(sample_rate);
    sirenOsc.SetWaveform(kSirenWaves[0]);
    sirenOsc.SetAmp(1.f);
    sirenLfo.Init(sample_rate);
    sirenLfo.SetWaveform(Oscillator::WAVE_SIN);
    sirenLfo.SetFreq(sirenRate);
    sirenLfo.SetAmp(0.f);
    sirenFbFilt.Init(sample_rate);
    sirenFbFilt.SetFreq(2800.f);
    sirenFbFilt.SetRes(0.2f);
    sirenFbFilt.SetDrive(0.1f);
    sirenDelay.Init();
    sirenDelTarget = sirenDelCurrent = sample_rate * 0.3f;
    sirenToneCoef  = 1.f - expf(-TWOPI_F * sirenToneHz / sample_rate);

    // start callback
    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    while(1) {}
}

// ---------------------------------------------------------------------------
// Effects-mode control handlers (unchanged behavior, except button RELEASE)
// ---------------------------------------------------------------------------
void UpdateKnobs(float &k1, float &k2)
{
    k1 = pod.knob1.Process();
    k2 = pod.knob2.Process();

    switch(mode)
    {
        case REV:
            drywet = k1;
            rev.SetFeedback(k2);
            break;
        case DEL:
            if(fabsf(k1 - knob1Last) > 0.02f)
            {
                delayTarget = deltime.Process();
                knob1Last   = k1;
            }
            feedback = k2;
            break;
        case CRU:
            cutoff = cutoffParam.Process();
            tone.SetFreq(cutoff);
            crushmod = (int)crushrate.Process();
            break;
        case FIL:
            filtFreq = filtFreqParam.Process();
            svfl.SetFreq(filtFreq);
            svfr.SetFreq(filtFreq);
            svfl.SetRes(k2);
            svfr.SetRes(k2);
            break;
        case PAN:
            panDepth = k2;
            if(panSynced)
            {
                // one full L-R sweep per delay repeat, whatever that's set to
                float syncFreq = sample_rate / currentDelay;
                panLfo.SetFreq(fclamp(syncFreq, 0.02f, 20.f));
            }
            else
            {
                panRate = fmap(k1, 0.05f, 8.f, Mapping::EXP);
                panLfo.SetFreq(panRate);
            }
            break;
    }
}

void UpdateButtons()
{
    // button 1 = previous effect, button 2 = next effect (5 effects now).
    // fire on RELEASE so the both-buttons combo can't step an effect first.
    if(pod.button1.FallingEdge())
        mode = (mode + 4) % 5; // step back (wraps 0 -> 4)
    if(pod.button2.FallingEdge())
        mode = (mode + 1) % 5; // step forward (wraps 4 -> 0)
}

void UpdateEncoderTurn()
{
    // PAN mode claims the turn for Shape; every other mode keeps the
    // original master wet/dry.
    if(mode == PAN)
    {
        panShape += pod.encoder.Increment() * 0.02f;
        panShape = fclamp(panShape, 0.0f, 1.0f);
    }
    else
    {
        masterWet += pod.encoder.Increment() * 0.02f;
        masterWet = fclamp(masterWet, 0.0f, 1.0f);
    }
}

void UpdateEncoderButton()
{
    if(pod.encoder.RisingEdge())
        pressStart = System::GetNow();

    if(pod.encoder.FallingEdge())
    {
        uint32_t held = System::GetNow() - pressStart;

        if(mode == PAN)
        {
            //   short press (< 500 ms) = cycle waveform
            //   long press  (>= 500 ms) = toggle free-Hz <-> synced to delay
            if(held >= 500)
                panSynced = !panSynced;
            else
            {
                panWave = (panWave + 1) % 3;
                panLfo.SetWaveform(kPanWaves[panWave]);
            }
            return;
        }

        //   short press (< 500 ms) = tap tempo for the delay
        //   long press  (>= 500 ms) = cycle filter type LP -> HP -> BP
        if(held >= 500)
        {
            filtType = (filtType + 1) % 3;
        }
        else
        {
            uint32_t now      = System::GetNow();
            uint32_t interval = now - lastTapTime;
            lastTapTime       = now;

            if(interval > 120 && interval < 2000)
            {
                float d = (interval * 0.001f) * sample_rate;
                if(d > (float)MAX_DELAY)
                    d = (float)MAX_DELAY;
                delayTarget = d;
            }
        }
    }
}

void UpdateLeds(float k1, float k2)
{
    if(mode == PAN)
    {
        // live pan meter: LED1 = left level, LED2 = right level (red)
        pod.led1.Set(panAmpL, 0.f, 0.f);
        pod.led2.Set(panAmpR, 0.f, 0.f);
        pod.UpdateLeds();
        return;
    }

    float r = 0, g = 0, b = 0;
    switch(mode)
    {
        case REV: b = 1;        break; // blue
        case DEL: g = 1;        break; // green
        case CRU: r = 1; b = 1; break; // magenta
        case FIL:
            switch(filtType)
            {
                case 0: r = 1; g = 1;        break; // LP = yellow
                case 1: g = 1; b = 1;        break; // HP = cyan
                case 2: r = 1; g = 1; b = 1; break; // BP = white
            }
            break;
    }
    pod.led1.Set(r * k1, g * k1, b * k1);
    pod.led2.Set(r * k2, g * k2, b * k2);
    pod.UpdateLeds();
}

// ---------------------------------------------------------------------------
// Both-buttons hold: toggle Effects <-> Siren
// ---------------------------------------------------------------------------
void HandleModeCombo()
{
    bool both = pod.button1.Pressed() && pod.button2.Pressed();

    if(both)
    {
        if(!comboActive)
        {
            comboActive = true;
            comboStart  = System::GetNow();
            comboFired  = false;
        }
        if(!comboFired && (System::GetNow() - comboStart >= 1000))
        {
            sirenMode    = !sirenMode;
            comboFired   = true;
            comboSwallow = true;
            // clean entry: no stuck note, not in level-edit
            sirenLatch     = false;
            sirenGate      = false;
            sirenLevelEdit = false;
        }
    }
    else
    {
        comboActive = false;
    }

    // stop swallowing once BOTH buttons are fully released
    if(comboSwallow && !pod.button1.Pressed() && !pod.button2.Pressed())
        comboSwallow = false;
}

// ---------------------------------------------------------------------------
// Siren-mode control handler
// ---------------------------------------------------------------------------
void SirenControls()
{
    // encoder click: short = flip page, long = next waveform
    static bool     sPrev  = false;
    static uint32_t sPress = 0;
    static bool     sLong  = false;
    bool sNow = pod.encoder.Pressed();
    if(sNow && !sPrev) { sPress = System::GetNow(); sLong = false; }
    if(sNow && !sLong && (System::GetNow() - sPress > 500))
    {
        if(sirenPage == 0) // wail page -> cycle the LFO (wail) shape
        {
            sirenLfoWave = (sirenLfoWave + 1) % 4;
            sirenLfo.SetWaveform(kLfoWaves[sirenLfoWave]);
            sirenBlinks = sirenLfoWave + 1; // blink out the position (1..4)
        }
        else               // echo page -> cycle the oscillator (voice) shape
        {
            sirenWave = (sirenWave + 1) % 4;
            sirenOsc.SetWaveform(kSirenWaves[sirenWave]);
            sirenBlinks = sirenWave + 1;    // blink out the position (1..4)
        }
        sirenBlinkOn = true;                     // start the first pulse now
        sirenBlinkT  = System::GetNow() + 90;
        sLong = true;
    }
    if(!sNow && sPrev && !sLong)
        sirenPage ^= 1;
    sPrev = sNow;

    // button 2:
    //   quick tap          -> latch on/off
    //   hold ~0.5s (alone)  -> toggle level-edit mode (LED1 magenta)
    // The long-press fires WHILE you hold (like the mode combo) so it's robust
    // to release bounce, and you see magenta before you let go. The "alone"
    // check (button 1 not held) keeps it clear of the both-buttons exit combo.
    static bool     b2Held  = false;
    static uint32_t b2Start = 0;
    static bool     b2Fired = false;
    bool b2Alone = pod.button2.Pressed() && !pod.button1.Pressed();

    if(b2Alone)
    {
        if(!b2Held) { b2Held = true; b2Start = System::GetNow(); b2Fired = false; }
        if(!b2Fired && (System::GetNow() - b2Start >= 500))
        {
            sirenLevelEdit = !sirenLevelEdit; // long press = level edit on/off
            b2Fired = true;
        }
    }
    else
    {
        // leaving the button2-alone state: a clean short tap (released, and the
        // long-press never fired) toggles latch
        if(b2Held && !b2Fired && !pod.button2.Pressed())
            sirenLatch = !sirenLatch;
        b2Held = false;
    }

    // button 1 = momentary gate
    sirenGate = pod.button1.Pressed() || sirenLatch;

    const int   e  = pod.encoder.Increment();
    const float k1 = pod.knob1.Value();
    const float k2 = pod.knob2.Value();

    if(sirenLevelEdit)
    {
        // encoder sets siren level vs. the passthrough audio (set & forget)
        sirenLevel = fclamp(sirenLevel + 0.03f * e, 0.f, 1.5f);
    }
    else if(sirenPage == 0)
    {
        sirenBaseFreq = fmap(k1, 30.f, 3000.f, Mapping::EXP);
        sirenRate     = fmap(k2, 0.1f, 20.f, Mapping::EXP);
        sirenDepth    = fclamp(sirenDepth + 15.f * e, 0.f, 1200.f);
    }
    else
    {
        sirenDelMs    = fmap(k1, 20.f, 2000.f, Mapping::EXP);
        sirenFeedback = fmap(k2, 0.f, 0.95f, Mapping::LINEAR);
        sirenToneHz   = fclamp(sirenToneHz * powf(1.06f, (float)e), 300.f, 12000.f);
    }

    sirenLfo.SetFreq(sirenRate);
    sirenLfo.SetAmp(sirenDepth);
    sirenDelTarget = (sirenDelMs / 1000.f) * sample_rate;
    sirenToneCoef  = 1.f - expf(-TWOPI_F * sirenToneHz / sample_rate);

    // LEDs: magenta = level edit, else page color; LED2 = sounding
    if(sirenLevelEdit)      pod.led1.Set(1.f, 0.f, 1.f); // magenta = level edit
    else if(sirenPage == 0) pod.led1.Set(0.f, 1.f, 0.f); // green   = wail page
    else                    pod.led1.Set(0.f, 0.f, 1.f); // blue    = echo page
    // LED2: while a count sequence runs, blink white N times (N = shape
    // position); otherwise show green while the siren is sounding.
    if(sirenBlinks > 0)
    {
        uint32_t now = System::GetNow();
        if(now >= sirenBlinkT)
        {
            if(sirenBlinkOn)
            {
                sirenBlinkOn = false;      // end of a pulse
                sirenBlinkT  = now + 130;  // gap
                sirenBlinks--;
            }
            else if(sirenBlinks > 0)
            {
                sirenBlinkOn = true;       // start next pulse
                sirenBlinkT  = now + 90;
            }
        }
        float v = sirenBlinkOn ? 1.f : 0.f;
        pod.led2.Set(v, v, v);
    }
    else
    {
        pod.led2.Set(0.f, sirenGate ? 1.f : 0.f, 0.f);
    }
    pod.UpdateLeds();
}

float GetSirenSample()
{
    fonepole(sirenAmp, sirenGate ? 1.f : 0.f, 0.002f); // click-free gate

    float f = fclamp(sirenBaseFreq + sirenLfo.Process(), 20.f, 8000.f);
    sirenOsc.SetFreq(f);
    float sig = sirenOsc.Process() * sirenAmp;

    fonepole(sirenToneState, sig, sirenToneCoef); // tone (output lowpass)
    sig = sirenToneState;

    fonepole(sirenDelCurrent, sirenDelTarget, 0.0002f); // slewed -> dub bend
    sirenDelay.SetDelay(sirenDelCurrent);

    float echo = sirenDelay.Read();
    sirenFbFilt.Process(echo);
    echo = sirenFbFilt.Low();
    sirenDelay.Write(tanhf(sig + sirenFeedback * echo)); // soft-clipped feedback

    return (sig + echo) * sirenLevel;
}

// ---------------------------------------------------------------------------
void Controls()
{
    float k1, k2;
    feedback = drywet = 0;

    pod.ProcessAnalogControls();
    pod.ProcessDigitalControls();

    HandleModeCombo();

    if(comboSwallow)
    {
        // mid mode-switch: hold both LEDs white, ignore normal button jobs
        pod.led1.Set(1.f, 1.f, 1.f);
        pod.led2.Set(1.f, 1.f, 1.f);
        pod.UpdateLeds();
        return;
    }

    if(sirenMode)
    {
        SirenControls();
    }
    else
    {
        UpdateKnobs(k1, k2);
        UpdateButtons();
        UpdateEncoderTurn();
        UpdateEncoderButton();
        UpdateLeds(k1, k2);
    }
}

void GetReverbSample(float &outl, float &outr, float inl, float inr)
{
    rev.Process(inl, inr, &outl, &outr);
    outl = drywet * outl + (1 - drywet) * inl;
    outr = drywet * outr + (1 - drywet) * inr;
}

void GetDelaySample(float &outl, float &outr, float inl, float inr)
{
    fonepole(currentDelay, delayTarget, .00007f);
    delr.SetDelay(currentDelay);
    dell.SetDelay(currentDelay);
    outl = dell.Read();
    outr = delr.Read();

    dell.Write((feedback * outl) + inl);
    outl = (feedback * outl) + ((1.0f - feedback) * inl);

    delr.Write((feedback * outr) + inr);
    outr = (feedback * outr) + ((1.0f - feedback) * inr);

    float wetl, wetr;
    rev.Process(outl, outr, &wetl, &wetr);
    outl = (1.0f - reverbSend) * outl + reverbSend * wetl;
    outr = (1.0f - reverbSend) * outr + reverbSend * wetr;
}

void GetCrushSample(float &outl, float &outr, float inl, float inr)
{
    crushcount++;
    crushcount %= crushmod;
    if(crushcount == 0)
    {
        crushsr = inr;
        crushsl = inl;
    }
    outl = tone.Process(crushsl);
    outr = tone.Process(crushsr);
}

void GetFilterSample(float &outl, float &outr, float inl, float inr)
{
    svfl.Process(inl);
    svfr.Process(inr);
    switch(filtType)
    {
        case 0: outl = svfl.Low();  outr = svfr.Low();  break; // lowpass
        case 1: outl = svfl.High(); outr = svfr.High(); break; // highpass
        case 2: outl = svfl.Band(); outr = svfr.Band(); break; // bandpass
        default: outl = svfl.Low(); outr = svfr.Low();
    }
}

void GetPanSample(float &outl, float &outr, float inl, float inr)
{
    // sum to mono, then sweep it across the stereo field - the classic
    // auto-pan behavior, and what you want for a mono dub siren voice.
    float mono = 0.5f * (inl + inr);

    float lfo = panLfo.Process(); // -1..+1

    // Shape: tanh-based sharpening. 0 = pure sine/tri/square, 1 = sweep
    // spends most of its time pinned near the extremes with a fast snap
    // between them (Ableton's Shape does this by pushing the waveform
    // toward its limits).
    float shaped = lfo;
    if(panShape > 0.001f)
    {
        float k = 1.f + panShape * 8.f;
        shaped  = tanhf(lfo * k) / tanhf(k);
    }

    float pos = fclamp(shaped * panDepth, -1.f, 1.f); // -1 (L) .. +1 (R)

    // equal-power pan law
    float angle = (pos + 1.f) * (PI_F * 0.25f); // 0 .. PI/2
    float ampL  = cosf(angle);
    float ampR  = sinf(angle);

    panAmpL = ampL;
    panAmpR = ampR;

    outl = mono * ampL;
    outr = mono * ampR;
}
