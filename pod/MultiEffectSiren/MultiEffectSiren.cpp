#include "daisysp.h"
#include "daisy_pod.h"
#include <math.h>

// ============================================================================
// MultiEffect + Dub Siren
//
// Chris's 4-effect MultiEffect (reverb / delay / crush / filter) with the dub
// siren grafted in as a FIFTH mode that lives OUTSIDE the effect rotation.
//
//   * Effects mode  : your four effects, cycled by the buttons, exactly as
//                     before (see note on button timing below).
//   * Siren mode    : the dub siren takes over. Buttons become triggers.
//
//   Toggle between the two: HOLD BUTTON 1 + BUTTON 2 together for ~1 second.
//   Both LEDs go white while you hold; release into the new mode's colors.
//
// NOTE: effect switching now fires on button RELEASE (FallingEdge) instead of
// press. This is what lets the both-buttons hold work without the first button
// stepping an effect mid-combo. Imperceptible in normal use.
// ============================================================================

// Set max delay time to 2.5 sec.
#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)
#define REV 0
#define DEL 1
#define CRU 2
#define FIL 3

using namespace daisysp;
using namespace daisy;

static DaisyPod pod;

static ReverbSc                                  rev;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS dell;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delr;
static Tone                                      tone;
static Svf                                       svfl, svfr;
static Parameter deltime, cutoffParam, crushrate, filtFreqParam;
int              mode = REV;

float currentDelay, feedback, delayTarget, cutoff;

int   crushmod, crushcount;
float crushsl, crushsr, drywet;

// --- added state (effects) ---
float    sample_rate;         // file-scope so tap tempo can convert ms -> samples
float    reverbSend = 0.25f;  // how much reverb rides on the delay (0..1)
float    masterWet  = 0.5f;   // encoder-turn master wet/dry (0..1)
uint32_t lastTapTime = 0;     // ms timestamp of previous encoder tap
float    knob1Last  = -1.0f;  // for detecting knob1 movement in DEL mode
float    filtFreq;            // current filter cutoff (Hz)
int      filtType   = 0;      // 0 = LP, 1 = HP, 2 = BP
uint32_t pressStart = 0;      // ms timestamp of encoder press (short vs long)

// --- dub siren (5th mode) ---
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
    }
}

void UpdateButtons()
{
    // button 1 = previous effect, button 2 = next effect.
    // fire on RELEASE so the both-buttons combo can't step an effect first.
    if(pod.button1.FallingEdge())
        mode = (mode + 3) % 4; // step back (wraps 0 -> 3)
    if(pod.button2.FallingEdge())
        mode = (mode + 1) % 4; // step forward (wraps 3 -> 0)
}

void UpdateEncoderTurn()
{
    masterWet += pod.encoder.Increment() * 0.02f;
    masterWet = fclamp(masterWet, 0.0f, 1.0f);
}

void UpdateEncoderButton()
{
    //   short press (< 500 ms) = tap tempo for the delay
    //   long press  (>= 500 ms) = cycle filter type LP -> HP -> BP
    if(pod.encoder.RisingEdge())
        pressStart = System::GetNow();

    if(pod.encoder.FallingEdge())
    {
        uint32_t held = System::GetNow() - pressStart;

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
