#include "daisysp.h"
#include "daisy_seed.h"
#include <math.h>

// ============================================================================
// MultiEffect + Dub Siren — BARE SEED VERSION (no DaisyPod class)
//
// For Daisy Seed on a breadboard wired to the Pod's pin layout:
//   Knob 1 = D21, Knob 2 = D15
//   Button 1 = D27, Button 2 = D28
//   Encoder A = D26, B = D25, Click = D13
//   (LEDs not required — onboard LED used for basic feedback)
//
// Same features as the Pod version:
//   * Effects mode  : reverb / delay / crush / filter, cycled by buttons.
//   * Siren mode    : dub siren, toggled by holding both buttons ~1 sec.
// ============================================================================

#define MAX_DELAY static_cast<size_t>(48000 * 2.5f)
#define REV 0
#define DEL 1
#define CRU 2
#define FIL 3

using namespace daisysp;
using namespace daisy;

// --- hardware objects (replaces DaisyPod) ---
static DaisySeed    hw;
static Switch       button1, button2;
static Encoder      encoder;
static AnalogControl knob1, knob2;

// --- DSP objects (unchanged) ---
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
float    sample_rate;
float    reverbSend = 0.25f;
float    masterWet  = 0.5f;
uint32_t lastTapTime = 0;
float    knob1Last  = -1.0f;
float    filtFreq;
int      filtType   = 0;
uint32_t pressStart = 0;

// --- dub siren (5th mode) ---
static Oscillator sirenOsc, sirenLfo;
static Svf        sirenFbFilt;
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS sirenDelay;

bool  sirenMode      = false;
int   sirenPage      = 0;
int   sirenWave      = 0;
int   sirenLfoWave   = 0;
bool  sirenLatch     = false;
bool  sirenGate      = false;
float sirenBaseFreq  = 220.f;
float sirenRate      = 4.f;
float sirenDepth     = 0.f;
float sirenDelMs     = 300.f;
float sirenFeedback  = 0.5f;
float sirenToneHz    = 4000.f;
float sirenDelTarget, sirenDelCurrent;
float sirenAmp       = 0.f;
float sirenToneState = 0.f;
float sirenToneCoef  = 0.f;
float sirenLevel     = 0.7f;
bool  sirenLevelEdit = false;
int      sirenBlinks  = 0;
bool     sirenBlinkOn = false;
uint32_t sirenBlinkT  = 0;

static const uint8_t kSirenWaves[4] = {
    Oscillator::WAVE_POLYBLEP_SQUARE,
    Oscillator::WAVE_POLYBLEP_SAW,
    Oscillator::WAVE_POLYBLEP_TRI,
    Oscillator::WAVE_SIN,
};

static const uint8_t kLfoWaves[4] = {
    Oscillator::WAVE_SIN,
    Oscillator::WAVE_TRI,
    Oscillator::WAVE_SQUARE,
    Oscillator::WAVE_RAMP,
};

// --- both-buttons combo state ---
uint32_t comboStart   = 0;
bool     comboActive  = false;
bool     comboFired   = false;
bool     comboSwallow = false;

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

    for(size_t i = 0; i < size; i += 2)
    {
        if(sirenMode)
        {
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

        out[i]     = (1.0f - masterWet) * inl + masterWet * outl;
        out[i + 1] = (1.0f - masterWet) * inr + masterWet * outr;
    }
}

int main(void)
{
    hw.Configure();
    hw.Init();
    hw.SetAudioBlockSize(4);
    sample_rate = hw.AudioSampleRate();

    // --- ADC: two knobs ---
    AdcChannelConfig adcCfg[2];
    adcCfg[0].InitSingle(hw.GetPin(21)); // knob1
    adcCfg[1].InitSingle(hw.GetPin(15)); // knob2
    hw.adc.Init(adcCfg, 2);
    hw.adc.Start();
    knob1.Init(hw.adc.GetPtr(0), hw.AudioCallbackRate());
    knob2.Init(hw.adc.GetPtr(1), hw.AudioCallbackRate());

    // --- Buttons (active low, internal pull-up) ---
    button1.Init(hw.GetPin(27), hw.AudioCallbackRate());
    button2.Init(hw.GetPin(28), hw.AudioCallbackRate());

    // --- Encoder ---
    encoder.Init(hw.GetPin(26), hw.GetPin(25), hw.GetPin(13),
                 hw.AudioCallbackRate());

    // --- DSP init ---
    rev.Init(sample_rate);
    dell.Init();
    delr.Init();
    tone.Init(sample_rate);
    svfl.Init(sample_rate);
    svfr.Init(sample_rate);

    // Parameter objects use knob1/knob2 AnalogControl
    deltime.Init(knob1, sample_rate * .05, MAX_DELAY, deltime.LOGARITHMIC);
    cutoffParam.Init(knob1, 500, 20000, cutoffParam.LOGARITHMIC);
    crushrate.Init(knob2, 1, 50, crushrate.LOGARITHMIC);
    filtFreqParam.Init(knob1, 60, 16000, filtFreqParam.LOGARITHMIC);

    rev.SetLpFreq(18000.0f);
    rev.SetFeedback(0.85f);

    currentDelay = delayTarget = sample_rate * 0.75f;
    dell.SetDelay(currentDelay);
    delr.SetDelay(currentDelay);

    // Siren init
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

    // 3 blinks = alive
    for(int i = 0; i < 3; i++)
    {
        hw.SetLed(true);
        System::Delay(120);
        hw.SetLed(false);
        System::Delay(120);
    }

    // Start audio
    hw.StartAudio(AudioCallback);

    while(1) {}
}

// ---------------------------------------------------------------------------
// Effects-mode control handlers
// ---------------------------------------------------------------------------
void UpdateKnobs(float &k1, float &k2)
{
    k1 = knob1.Process();
    k2 = knob2.Process();

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
    if(button1.FallingEdge())
        mode = (mode + 3) % 4;
    if(button2.FallingEdge())
        mode = (mode + 1) % 4;
}

void UpdateEncoderTurn()
{
    masterWet += encoder.Increment() * 0.02f;
    masterWet = fclamp(masterWet, 0.0f, 1.0f);
}

void UpdateEncoderButton()
{
    if(encoder.RisingEdge())
        pressStart = System::GetNow();

    if(encoder.FallingEdge())
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
    // onboard LED: on in effects mode, off in siren (basic feedback)
    hw.SetLed(!sirenMode);
}

// ---------------------------------------------------------------------------
// Both-buttons hold: toggle Effects <-> Siren
// ---------------------------------------------------------------------------
void HandleModeCombo()
{
    bool both = button1.Pressed() && button2.Pressed();

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
            sirenLatch     = false;
            sirenGate      = false;
            sirenLevelEdit = false;
        }
    }
    else
    {
        comboActive = false;
    }

    if(comboSwallow && !button1.Pressed() && !button2.Pressed())
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
    bool sNow = encoder.Pressed();
    if(sNow && !sPrev) { sPress = System::GetNow(); sLong = false; }
    if(sNow && !sLong && (System::GetNow() - sPress > 500))
    {
        if(sirenPage == 0)
        {
            sirenLfoWave = (sirenLfoWave + 1) % 4;
            sirenLfo.SetWaveform(kLfoWaves[sirenLfoWave]);
            sirenBlinks = sirenLfoWave + 1;
        }
        else
        {
            sirenWave = (sirenWave + 1) % 4;
            sirenOsc.SetWaveform(kSirenWaves[sirenWave]);
            sirenBlinks = sirenWave + 1;
        }
        sirenBlinkOn = true;
        sirenBlinkT  = System::GetNow() + 90;
        sLong = true;
    }
    if(!sNow && sPrev && !sLong)
        sirenPage ^= 1;
    sPrev = sNow;

    // button 2: quick tap = latch, hold ~0.5s = level-edit toggle
    static bool     b2Held  = false;
    static uint32_t b2Start = 0;
    static bool     b2Fired = false;
    bool b2Alone = button2.Pressed() && !button1.Pressed();

    if(b2Alone)
    {
        if(!b2Held) { b2Held = true; b2Start = System::GetNow(); b2Fired = false; }
        if(!b2Fired && (System::GetNow() - b2Start >= 500))
        {
            sirenLevelEdit = !sirenLevelEdit;
            b2Fired = true;
        }
    }
    else
    {
        if(b2Held && !b2Fired && !button2.Pressed())
            sirenLatch = !sirenLatch;
        b2Held = false;
    }

    // button 1 = momentary gate
    sirenGate = button1.Pressed() || sirenLatch;

    const int   e  = encoder.Increment();
    const float k1 = knob1.Process();
    const float k2 = knob2.Process();

    if(sirenLevelEdit)
    {
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

    // onboard LED: on while siren is sounding
    hw.SetLed(sirenGate);
}

float GetSirenSample()
{
    fonepole(sirenAmp, sirenGate ? 1.f : 0.f, 0.002f);

    float f = fclamp(sirenBaseFreq + sirenLfo.Process(), 20.f, 8000.f);
    sirenOsc.SetFreq(f);
    float sig = sirenOsc.Process() * sirenAmp;

    fonepole(sirenToneState, sig, sirenToneCoef);
    sig = sirenToneState;

    fonepole(sirenDelCurrent, sirenDelTarget, 0.0002f);
    sirenDelay.SetDelay(sirenDelCurrent);

    float echo = sirenDelay.Read();
    sirenFbFilt.Process(echo);
    echo = sirenFbFilt.Low();
    sirenDelay.Write(tanhf(sig + sirenFeedback * echo));

    return (sig + echo) * sirenLevel;
}

// ---------------------------------------------------------------------------
void Controls()
{
    float k1, k2;
    feedback = drywet = 0;

    // debounce digital inputs
    button1.Debounce();
    button2.Debounce();
    encoder.Debounce();

    HandleModeCombo();

    if(comboSwallow)
        return;

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
        case 0: outl = svfl.Low();  outr = svfr.Low();  break;
        case 1: outl = svfl.High(); outr = svfr.High(); break;
        case 2: outl = svfl.Band(); outr = svfr.Band(); break;
        default: outl = svfl.Low(); outr = svfr.Low();
    }
}
