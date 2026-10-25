// Replacement for the Teensy Audio library graph used by OnSpeed Gen2:
//   AudioSynthWaveformSine -> AudioEffectEnvelope -> mixer -> ampLeft/ampRight -> DACs
// plus the IntervalTimer-driven tonePlayHandler()/setFrequencytone() from Tones.ino.
// Renders interleaved int16 stereo PCM for an I2S DAC (PCM5102A etc.).
#pragma once
#include <stdint.h>
#include "onspeed_core.h"

namespace onspeed {

struct ToneEvent {      // emitted for test/telemetry: start of each tone envelope
  uint32_t sampleIndex;
  uint16_t freq;
  uint8_t mode;         // PULSE_TONE or SOLID_TONE
  float volume;
};

class ToneSynth {
 public:
  explicit ToneSynth(int sampleRate);
  // Called from the 50 Hz sensor task after Core::updateTones().
  void setToneState(ToneMode mode, bool highTone, float pps);
  void setMasterVolume(float v) { master_ = v; }
  // Check3DAudio() from 3DAudio.ino: lateral G pans the tone with the slip ball.
  void update3DAudio(float lateralG);
  // Render n stereo frames into out[2*n].
  void render(int16_t *out, int n);
  // Pop the next tone onset event; returns false if none.
  bool popEvent(ToneEvent &e);
  float gainLeft() const { return gainL_; }
  float gainRight() const { return gainR_; }
  uint32_t samplesRendered() const { return sampleIndex_; }

 private:
  void tonePlayHandler();
  void setFrequencytone(uint32_t frequency);
  void noteOn(float delayMs, float attackMs, float holdMs, float decayMs, float sustain, float releaseMs);
  void noteOff();
  int ms2s(float ms) const { return ms <= 0 ? 0 : (int)(ms * rate_ / 1000.0f); }

  int rate_;
  ToneMode toneMode_ = TONE_OFF;
  bool highTone_ = false;
  float pps_ = 20;
  ToneMode tonePlaying_ = TONE_OFF;
  uint32_t toneFreq_ = 0;
  // timer
  uint32_t sampleIndex_ = 0;
  int64_t nextHandler_ = 0;
  // oscillator
  uint32_t phase_ = 0, phaseInc_ = 0;
  float amplitude_ = 0;
  // envelope
  enum EnvState { ENV_IDLE, ENV_DELAY, ENV_ATTACK, ENV_HOLD, ENV_DECAY, ENV_SUSTAIN, ENV_RELEASE };
  EnvState env_ = ENV_IDLE;
  int envCount_ = 0;
  float envLevel_ = 0, envStep_ = 0;
  int dly_ = 0, att_ = 0, hold_ = 0, dec_ = 0, rel_ = 0;
  float sus_ = 0;
  // output
  float master_ = 1.0f, channelGain_ = 0, gainL_ = 1, gainR_ = 1;
  ToneEvent ev_[32];
  int evHead_ = 0, evTail_ = 0;
};

}  // namespace onspeed
