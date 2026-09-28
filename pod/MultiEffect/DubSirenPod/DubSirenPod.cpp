// DubSirenPod.cpp
// A playable dub siren for the stock Electrosmith Daisy Pod
//   (2 pots, 1 clickable encoder, 2 buttons, 2 RGB LEDs).
//
// Voice DSP adapted from beserge/DubSiren (band-limited square + sine-LFO
// pitch wail + slewed feedback delay). Reworked here with a two-page control
// scheme, click-free gate, latch, selectable waveform, an output tone
// control, and a soft-clipped feedback write so the echo can't blow up.
// Original DSP credit: beserge (no explicit license on that repo — check
// before redistributing).
//
// CONTROLS
//   Pot 1    : page A -> pitch       | page B -> delay time
//   Pot 2    : page A -> wail rate    | page B -> feedback
//   Encoder  turn : page A -> wail depth | page B -> tone (output LP cutoff)
//            click (short) : flip page   (LED1 green = A / blue = B)
//            click (long)  : next waveform (square->saw->tri->sine)
//   Button 1 : momentary  (hold to sound)
//   Button 2 : latch      (tap to sustain, tap to stop)
//
// The two pots are absolute, so they jump to their new value on a page flip;
// the encoder is relative, so wail depth / tone hold across flips.

#include "daisy_pod.h"
#include "daisysp.h"
#include <cmath>

using namespace daisy;
using namespace daisysp;

DaisyPod pod;

static constexpr size_t MAX_DELAY = static_cast<size_t>(48000 * 2.5f);

Oscillator osc, lfo;
Svf        fbFilt; // fixed-dark filter inside the echo feedback path
static DelayLine<float, MAX_DELAY> DSY_SDRAM_BSS delay;

// POLYBLEP waveforms are band-limited — the built-in fix for aliasing/rasp.
static const uint8_t kWaves[4] = {
    Oscillator::WAVE_POLYBLEP_SQUARE,
    Oscillator::WAVE_POLYBLEP_SAW,
    Oscillator::WAVE_POLYBLEP_TRI,
    Oscillator::WAVE_SIN,
};

// --- shared state ---
float sr        = 48000.f;
int   page      = 0;
int   waveIdx   = 0;
bool  latched   = false;
bool  gateOn    = false;

float baseFreq  = 220.f;  // Hz (pot, page A)
float lfoRate   = 4.f;    // Hz (pot, page A)
float lfoDepth  = 0.f;    // Hz of pitch deviation (encoder, page A)
float delMs     = 300.f;  // ms (pot, page B)
float feedback  = 0.5f;   // 0..1 (pot, page B)
float toneHz    = 4000.f; // output LP cutoff (encoder, page B)

float delTarget, delCurrent;
float amp       = 0.f;    // click-free gate envelope
float toneState = 0.f;    // output one-pole LP state

void UpdateControls()
{
    pod.ProcessAllControls();

    // encoder click: short press = flip page, long press = next waveform
    static bool     prevP     = false;
    static uint32_t pressT    = 0;
    static bool     longFired = false;
    bool nowP = pod.encoder.Pressed();
    if(nowP && !prevP) { pressT = System::GetNow(); longFired = false; }
    if(nowP && !longFired && (System::GetNow() - pressT > 500))
    {
        waveIdx = (waveIdx + 1) % 4;
        osc.SetWaveform(kWaves[waveIdx]);
        longFired = true;
    }
    if(!nowP && prevP && !longFired)
        page ^= 1;
    prevP = nowP;

    // buttons: momentary + latch
    if(pod.button2.RisingEdge())
        latched = !latched;
    gateOn = pod.button1.Pressed() || latched;

    const int   e  = pod.encoder.Increment(); // relative: no jump on flip
    const float k1 = pod.knob1.Value();
    const float k2 = pod.knob2.Value();

    if(page == 0)
    {
        baseFreq = fmap(k1, 30.f, 3000.f, Mapping::EXP);
        lfoRate  = fmap(k2, 0.1f, 20.f, Mapping::EXP);
        lfoDepth = fclamp(lfoDepth + 15.f * e, 0.f, 1200.f);
        pod.led1.Set(0.f, 1.f, 0.f);  // green = page A
    }
    else
    {
        delMs    = fmap(k1, 20.f, 2000.f, Mapping::EXP);
        feedback = fmap(k2, 0.f, 0.95f, Mapping::LINEAR);
        toneHz   = fclamp(toneHz * powf(1.06f, (float)e), 300.f, 12000.f);
        pod.led1.Set(0.f, 0.f, 1.f);  // blue = page B
    }

    lfo.SetFreq(lfoRate);
    lfo.SetAmp(lfoDepth);
    delTarget = (delMs / 1000.f) * sr;

    pod.led2.Set(0.f, gateOn ? 1.f : 0.f, 0.f); // lit while sounding
    pod.UpdateLeds();
}

void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    UpdateControls();
    const float toneCoef = 1.f - expf(-TWOPI_F * toneHz / sr);

    for(size_t i = 0; i < size; i++)
    {
        fonepole(amp, gateOn ? 1.f : 0.f, 0.002f); // click-free gate

        float f = fclamp(baseFreq + lfo.Process(), 20.f, 8000.f);
        osc.SetFreq(f);
        float sig = osc.Process() * amp;

        fonepole(toneState, sig, toneCoef); // tone knob (output lowpass)
        sig = toneState;

        fonepole(delCurrent, delTarget, 0.0002f); // slewed -> dub pitch-bend
        delay.SetDelay(delCurrent);

        float echo = delay.Read();
        fbFilt.Process(echo);
        echo = fbFilt.Low();
        delay.Write(std::tanh(sig + feedback * echo)); // soft-clipped feedback

        float mix = sig + echo;
        out[0][i] = out[1][i] = mix * 0.7f;
    }
}

int main(void)
{
    pod.Init();
    pod.SetAudioBlockSize(4);
    sr = pod.AudioSampleRate();

    delay.Init();
    delTarget = delCurrent = sr * 0.3f;

    osc.Init(sr);
    osc.SetWaveform(kWaves[0]);
    osc.SetAmp(1.f);

    lfo.Init(sr);
    lfo.SetWaveform(Oscillator::WAVE_SIN);
    lfo.SetFreq(lfoRate);
    lfo.SetAmp(0.f);

    fbFilt.Init(sr);
    fbFilt.SetFreq(2800.f);
    fbFilt.SetRes(0.2f);
    fbFilt.SetDrive(0.1f);

    pod.StartAdc();
    pod.StartAudio(AudioCallback);

    while(1) {}
}
