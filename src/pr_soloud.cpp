#include "pr_soloud.hpp"
#include <alsoundsystem.hpp>
#include <alsound_source.hpp>
#include <alsound_buffer.hpp>
#include <alsound_listener.hpp>
#include <sharedutils/util_pragma.hpp>
#include <soloud.h>
#include <soloud_wav.h>

namespace al {
	enum class SoloudError : uint32_t {
		NoError = SoLoud::SO_NO_ERROR,                // No error
		InvalidParameter = SoLoud::INVALID_PARAMETER, // Some parameter is invalid
		FileNotFound = SoLoud::FILE_NOT_FOUND,        // File not found
		FileLoadFailed = SoLoud::FILE_LOAD_FAILED,    // File found, but could not be loaded
		DllNotFound = SoLoud::DLL_NOT_FOUND,          // DLL not found, or wrong DLL
		OutOfMemory = SoLoud::OUT_OF_MEMORY,          // Out of memory
		NotImplemented = SoLoud::NOT_IMPLEMENTED,     // Feature not implemented
		UnknownError = SoLoud::UNKNOWN_ERROR          // Other error
	};

	class SoloudSoundSystem : public ISoundSystem {
	  public:
		SoloudSoundSystem(float metersPerUnit);
		~SoloudSoundSystem() override;
		SoloudError Initialize();

		virtual PEffect CreateEffect() { return nullptr; }
		virtual IAuxiliaryEffectSlot *CreateAuxiliaryEffectSlot() override { return nullptr; }

		virtual PDecoder CreateDecoder(const std::string &path, bool bConvertToMono = false) override { return nullptr; }
		virtual bool IsSupported(ChannelConfig channels, SampleType type) const override { return true; }

		virtual float GetDopplerFactor() const override { return m_dopplerFactor; }
		virtual void SetDopplerFactor(float factor) override { m_dopplerFactor = factor; }

		virtual float GetSpeedOfSound() const override { return m_speedOfSound; }
		virtual void SetSpeedOfSound(float speed) override;

		virtual DistanceModel GetDistanceModel() const override { return m_distanceModel; }
		virtual void SetDistanceModel(DistanceModel mdl) override { m_distanceModel = mdl; }

		virtual std::string GetDeviceName() const override { return "dummy"; }
		virtual void PauseDeviceDSP() override {}
		virtual void ResumeDeviceDSP() override {}

		virtual std::vector<std::string> GetDevices() override { return {"dummy"}; }
		virtual std::string GetDefaultDeviceName() override { return "dummy"; }

		// HRTF
		virtual std::vector<std::string> GetHRTFNames() const override { return {}; }
		virtual std::string GetCurrentHRTF() const override { return ""; }
		virtual bool IsHRTFEnabled() const override { return false; }
		virtual void SetHRTF(uint32_t id) override {}
		virtual void DisableHRTF() override {}

		virtual uint32_t GetMaxAuxiliaryEffectsPerSource() const { return 0; }
		SoLoud::Soloud &GetSoloudEngine() { return m_soloud; }
	  private:
		virtual ISoundBuffer *DoLoadSound(const std::string &path, bool bConvertToMono = false, bool bAsync = true) override;
		virtual PSoundChannel CreateChannel(ISoundBuffer &buffer) override;
		virtual PSoundChannel CreateChannel(Decoder &decoder) override { return nullptr; }
		virtual std::unique_ptr<IListener> CreateListener() override;
		SoLoud::Soloud m_soloud;
	};

	class SoloudSoundChannel : public ISoundChannel {
	  public:
		enum class State : uint8_t { Initial = 0, Playing, Stopped, Paused };
		SoloudSoundChannel(ISoundSystem &system, ISoundBuffer &buffer, SoLoud::WavInstance &&instance) : ISoundChannel {system, buffer}, m_instance {std::move(instance)} {}
		SoloudSoundChannel(ISoundSystem &system, Decoder &decoder, SoLoud::WavInstance &&instance) : ISoundChannel {system, decoder}, m_instance {std::move(instance)} {}

		virtual void Play() override;
		virtual void Stop() override;
		virtual void Pause() override;
		virtual void Resume() override;
		virtual bool IsPlaying() const override;
		virtual bool IsPaused() const override;
		virtual void SetPriority(uint32_t priority) override;
		virtual uint32_t GetPriority() const override;
		virtual void SetFrameOffset(uint64_t offset) override;
		virtual uint64_t GetFrameOffset(uint64_t *latency = nullptr) const override;
		virtual void SetLooping(bool bLoop) override;
		virtual bool IsLooping() const override;

		virtual void SetPitch(float pitch) override;
		virtual float GetPitch() const override;

		virtual void SetGain(float gain) override;
		virtual float GetGain() const override;

		virtual void SetGainRange(float minGain, float maxGain) override;
		virtual std::pair<float, float> GetGainRange() const override;
		virtual float GetMinGain() const override;
		virtual float GetMaxGain() const override;
		virtual void SetDistanceRange(float refDist, float maxDist) override;
		virtual std::pair<float, float> GetDistanceRange() const override;
		virtual void SetPosition(const Vector3 &pos) override;
		virtual Vector3 GetPosition() const override;
		virtual void SetVelocity(const Vector3 &vel) override;
		virtual Vector3 GetVelocity() const override;

		virtual void SetDirection(const Vector3 &dir) override;
		virtual Vector3 GetDirection() const override;

		virtual void SetOrientation(const Vector3 &at, const Vector3 &up) override;
		virtual std::pair<Vector3, Vector3> GetOrientation() const override;

		virtual void SetConeAngles(float inner, float outer) override;
		virtual std::pair<float, float> GetConeAngles() const override;
		virtual void SetOuterConeGains(float gain, float gainHF = 1.f) override;
		virtual std::pair<float, float> GetOuterConeGains() const override;
		virtual float GetOuterConeGain() const override;
		virtual float GetOuterConeGainHF() const override;

		virtual void SetRolloffFactors(float factor, float roomFactor = 0.f) override;
		virtual std::pair<float, float> GetRolloffFactors() const override;
		virtual float GetRolloffFactor() const override;
		virtual float GetRoomRolloffFactor() const override;

		virtual void SetDopplerFactor(float factor) override;
		virtual float GetDopplerFactor() const override;

		virtual void SetRelative(bool bRelative) override;
		virtual bool IsRelative() const override;

		virtual void SetRadius(float radius) override;
		virtual float GetRadius() const override;

		virtual void SetStereoAngles(float leftAngle, float rightAngle) override;
		virtual std::pair<float, float> GetStereoAngles() const override;
		virtual void SetAirAbsorptionFactor(float factor) override;
		virtual float GetAirAbsorptionFactor() const override;

		virtual void SetGainAuto(bool directHF, bool send, bool sendHF) override;
		virtual std::tuple<bool, bool, bool> GetGainAuto() const override;
		virtual bool GetDirectGainHFAuto() const override;
		virtual bool GetSendGainAuto() const override;
		virtual bool GetSendGainHFAuto() const override;

		virtual void SetDirectFilter(const EffectParams &params) override;
		virtual void SetEffectParameters(uint32_t slotId, const EffectParams &params) override;

		SoLoud::Soloud &GetSoloudEngine() { return static_cast<SoloudSoundSystem &>(m_system).GetSoloudEngine(); }
	  private:
		virtual void DoAddEffect(IAuxiliaryEffectSlot &slot, uint32_t slotId, const EffectParams &params) override {}
		virtual void DoRemoveInternalEffect(uint32_t slotId) override {}
		virtual void DoRemoveEffect(uint32_t slotId) override {}
		SoLoud::WavInstance m_instance;
		SoLoud::handle m_handle = 0;
		bool m_looping = false;
		bool m_relative = false;
		float m_refDist = 0.f;
		float m_maxDist = util::pragma::metres_to_units(1.0);
		uint32_t m_priority = 0;
		float m_radius = 0.f;
		float m_pitch = 1.f;
		float m_gain = 1.f;
		float m_outerConeGain = 1.f;
		float m_outerConeGainHf = 1.f;
		float m_dopplerFactor = 1.f;
		float m_rolloffFactor = 1.f;
		float m_roomRolloffFactor = 1.f;
		float m_airAbsorptionFactor = 1.f;
		float m_stereoLeftAngle = 0.f;
		float m_stereoRightAngle = 0.f;
		bool m_directGainHfAuto = false;
		bool m_sendGainAuto = false;
		bool m_sendGainHfAuto = false;
		float m_minGain = 0.f;
		float m_maxGain = 1.f;
		Vector3 m_pos {};
		Vector3 m_velocity {};
		Vector3 m_direction {0.f, 0.f, 0.f};
		Vector3 m_at {};
		Vector3 m_up {};
		float m_innerConeAngles = 0.f;
		float m_outerConeAngles = 0.f;
		State m_state = State::Initial;
	};

	class SoloudListener : public IListener {
	  public:
		SoloudListener(al::ISoundSystem &system) : IListener {system} {}

		virtual void SetGain(float gain) override;
		virtual void SetPosition(const Vector3 &pos) override;
		virtual void SetVelocity(const Vector3 &vel) override;
		virtual void SetOrientation(const Vector3 &at, const Vector3 &up) override;
	  protected:
		virtual void DoSetMetersPerUnit(float mu) override {}
	};

	class SoloudSoundBuffer : public ISoundBuffer {
	  public:
		SoloudSoundBuffer();
		virtual uint32_t GetFrequency() const override { return 0; }
		virtual ChannelConfig GetChannelConfig() const override { return ChannelConfig::Mono; }
		virtual SampleType GetSampleType() const override { return SampleType::Float32; }
		virtual uint64_t GetLength() const override { return 0; }
		virtual std::pair<uint64_t, uint64_t> GetLoopFramePoints() const override { return {}; }

		virtual bool IsReady() const override { return true; }

		virtual uint32_t GetSize() const override { return 0; }
		virtual void SetLoopFramePoints(uint32_t start, uint32_t end) override {}
		virtual void SetLoopTimePoints(float tStart, float tEnd) override {}

		virtual std::string GetName() const override { return ""; }
		virtual bool IsInUse() const override { return false; }

		SoLoud::Wav &GetSoloudWave() { return m_wave; }
	  private:
		SoLoud::Wav m_wave;
	};
};

al::SoloudSoundSystem::SoloudSoundSystem(float metersPerUnit) : ISoundSystem {metersPerUnit} {}

al::SoloudSoundSystem::~SoloudSoundSystem() { m_soloud.deinit(); }

al::SoloudError al::SoloudSoundSystem::Initialize() { return static_cast<SoloudError>(m_soloud.init()); }

void al::SoloudSoundChannel::Play()
{
	m_state = State::Playing;
	if(m_buffer.expired())
		return;
	auto &wave = static_cast<SoloudSoundBuffer *>(m_buffer.lock().get())->GetSoloudWave();
	m_handle = GetSoloudEngine().play(wave, GetGain(), 0.f, false);
}
void al::SoloudSoundChannel::Stop()
{
	m_state = State::Stopped;
	if(m_handle) {
		GetSoloudEngine().stop(m_handle);
		m_handle = 0;
	}
}
void al::SoloudSoundChannel::Pause()
{
	m_state = State::Paused;
	if(m_handle)
		GetSoloudEngine().setPause(m_handle, true);
}
void al::SoloudSoundChannel::Resume() { Play(); }
bool al::SoloudSoundChannel::IsPlaying() const { return m_state == State::Playing; }
bool al::SoloudSoundChannel::IsPaused() const { return m_state == State::Paused; }
void al::SoloudSoundChannel::SetPriority(uint32_t priority) { m_priority = priority; }
uint32_t al::SoloudSoundChannel::GetPriority() const { return m_priority; }
void al::SoloudSoundChannel::SetFrameOffset(uint64_t offset) {}
uint64_t al::SoloudSoundChannel::GetFrameOffset(uint64_t *latency) const { return 0; }
void al::SoloudSoundChannel::SetLooping(bool bLoop)
{
	m_looping = bLoop;
	//GetSoloudEngine().setLooping(m_handle);
}
bool al::SoloudSoundChannel::IsLooping() const { return m_looping; }

void al::SoloudSoundChannel::SetPitch(float pitch)
{
	m_pitch = pitch;
	// TODO: Not yet implemented
	// Can be implemented via FFTFilter https://github.com/jarikomppa/soloud/issues/261
}
float al::SoloudSoundChannel::GetPitch() const { return m_pitch; }

void al::SoloudSoundChannel::SetGain(float gain)
{
	m_gain = gain;
	if(m_handle)
		GetSoloudEngine().setVolume(m_handle, gain);
}
float al::SoloudSoundChannel::GetGain() const { return m_gain; }

void al::SoloudSoundChannel::SetGainRange(float minGain, float maxGain)
{
	m_minGain = minGain;
	m_maxGain = maxGain;
}
std::pair<float, float> al::SoloudSoundChannel::GetGainRange() const { return {m_minGain, m_maxGain}; }
float al::SoloudSoundChannel::GetMinGain() const { return m_minGain; }
float al::SoloudSoundChannel::GetMaxGain() const { return m_maxGain; }
void al::SoloudSoundChannel::SetDistanceRange(float refDist, float maxDist)
{
	m_refDist = refDist;
	m_maxDist = maxDist;
}
std::pair<float, float> al::SoloudSoundChannel::GetDistanceRange() const { return {m_refDist, m_maxDist}; }
void al::SoloudSoundChannel::SetPosition(const Vector3 &pos)
{
	m_pos = pos;
	if(m_handle)
		GetSoloudEngine().set3dSourcePosition(m_handle, pos.x, pos.y, pos.z);
}
Vector3 al::SoloudSoundChannel::GetPosition() const { return m_pos; }
void al::SoloudSoundChannel::SetVelocity(const Vector3 &vel)
{
	m_velocity = vel;
	if(m_handle)
		GetSoloudEngine().set3dSourceVelocity(m_handle, vel.x, vel.y, vel.z);
}
Vector3 al::SoloudSoundChannel::GetVelocity() const { return m_velocity; }

void al::SoloudSoundChannel::SetDirection(const Vector3 &dir) { m_direction = dir; }
Vector3 al::SoloudSoundChannel::GetDirection() const { return m_direction; }

void al::SoloudSoundChannel::SetOrientation(const Vector3 &at, const Vector3 &up)
{
	m_at = at;
	m_up = up;
}
std::pair<Vector3, Vector3> al::SoloudSoundChannel::GetOrientation() const { return {m_at, m_up}; }

void al::SoloudSoundChannel::SetConeAngles(float inner, float outer)
{
	m_innerConeAngles = inner;
	m_outerConeAngles = outer;
}
std::pair<float, float> al::SoloudSoundChannel::GetConeAngles() const { return {m_innerConeAngles, m_outerConeAngles}; }
void al::SoloudSoundChannel::SetOuterConeGains(float gain, float gainHF)
{
	m_outerConeGain = gain;
	m_outerConeGainHf = gainHF;
}
std::pair<float, float> al::SoloudSoundChannel::GetOuterConeGains() const { return {m_outerConeGain, m_outerConeGainHf}; }
float al::SoloudSoundChannel::GetOuterConeGain() const { return m_outerConeGain; }
float al::SoloudSoundChannel::GetOuterConeGainHF() const { return m_outerConeGainHf; }

void al::SoloudSoundChannel::SetRolloffFactors(float factor, float roomFactor)
{
	m_rolloffFactor = factor;
	m_roomRolloffFactor = roomFactor;
}
std::pair<float, float> al::SoloudSoundChannel::GetRolloffFactors() const { return {m_rolloffFactor, m_roomRolloffFactor}; }
float al::SoloudSoundChannel::GetRolloffFactor() const { return m_rolloffFactor; }
float al::SoloudSoundChannel::GetRoomRolloffFactor() const { return m_roomRolloffFactor; }

void al::SoloudSoundChannel::SetDopplerFactor(float factor) { m_dopplerFactor = factor; }
float al::SoloudSoundChannel::GetDopplerFactor() const { return m_dopplerFactor; }

void al::SoloudSoundChannel::SetRelative(bool bRelative) { m_relative = bRelative; }
bool al::SoloudSoundChannel::IsRelative() const { return m_relative; }

void al::SoloudSoundChannel::SetRadius(float radius)
{
	m_radius = radius;
	if(m_handle)
		GetSoloudEngine().set3dSourceMinMaxDistance(m_handle, GetReferenceDistance(), radius);
}
float al::SoloudSoundChannel::GetRadius() const { return m_radius; }

void al::SoloudSoundChannel::SetStereoAngles(float leftAngle, float rightAngle)
{
	m_stereoLeftAngle = leftAngle;
	m_stereoRightAngle = rightAngle;
}
std::pair<float, float> al::SoloudSoundChannel::GetStereoAngles() const { return {m_stereoLeftAngle, m_stereoRightAngle}; }
void al::SoloudSoundChannel::SetAirAbsorptionFactor(float factor) { m_airAbsorptionFactor = factor; }
float al::SoloudSoundChannel::GetAirAbsorptionFactor() const { return m_airAbsorptionFactor; }

void al::SoloudSoundChannel::SetGainAuto(bool directHF, bool send, bool sendHF)
{
	m_directGainHfAuto = directHF;
	m_sendGainAuto = send;
	m_sendGainHfAuto = sendHF;
}
std::tuple<bool, bool, bool> al::SoloudSoundChannel::GetGainAuto() const { return {m_directGainHfAuto, m_sendGainAuto, m_sendGainHfAuto}; }
bool al::SoloudSoundChannel::GetDirectGainHFAuto() const { return m_directGainHfAuto; }
bool al::SoloudSoundChannel::GetSendGainAuto() const { return m_sendGainAuto; }
bool al::SoloudSoundChannel::GetSendGainHFAuto() const { return m_sendGainHfAuto; }

void al::SoloudSoundChannel::SetDirectFilter(const EffectParams &params) {}
void al::SoloudSoundChannel::SetEffectParameters(uint32_t slotId, const EffectParams &params) {}

void al::SoloudSoundSystem::SetSpeedOfSound(float speed) { m_soloud.set3dSoundSpeed(speed); }

al::PSoundChannel al::SoloudSoundSystem::CreateChannel(ISoundBuffer &buffer)
{
	SoLoud::WavInstance instance {&static_cast<SoloudSoundBuffer &>(buffer).GetSoloudWave()};
	return std::make_shared<SoloudSoundChannel>(*this, buffer, std::move(instance));
}
al::ISoundBuffer *al::SoloudSoundSystem::DoLoadSound(const std::string &path, bool bConvertToMono, bool bAsync)
{
	auto buf = std::make_shared<SoloudSoundBuffer>();
	auto &wave = buf->GetSoloudWave();
	auto res = static_cast<SoloudError>(wave.load(path.c_str()));
	if(res != al::SoloudError::NoError)
		return nullptr;
	if(buf->GetChannelConfig() == al::ChannelConfig::Mono || bConvertToMono == true)
		m_buffers[path].mono = buf;
	else
		m_buffers[path].stereo = buf;
	return buf.get();
}
std::unique_ptr<al::IListener> al::SoloudSoundSystem::CreateListener() { return std::make_unique<SoloudListener>(*this); }

///////////////

al::SoloudSoundBuffer::SoloudSoundBuffer() {}

///////////////

void al::SoloudListener::SetGain(float gain)
{
	// TODO
}
void al::SoloudListener::SetPosition(const Vector3 &pos) { static_cast<SoloudSoundSystem &>(m_soundSystem).GetSoloudEngine().set3dListenerPosition(pos.x, pos.y, pos.z); }
void al::SoloudListener::SetVelocity(const Vector3 &vel) { static_cast<SoloudSoundSystem &>(m_soundSystem).GetSoloudEngine().set3dListenerVelocity(vel.x, vel.y, vel.z); }
void al::SoloudListener::SetOrientation(const Vector3 &at, const Vector3 &up)
{
	static_cast<SoloudSoundSystem &>(m_soundSystem).GetSoloudEngine().set3dListenerAt(at.x, at.y, at.z);
	static_cast<SoloudSoundSystem &>(m_soundSystem).GetSoloudEngine().set3dListenerUp(up.x, up.y, up.z);
}

#ifdef __linux__
#define DLLEXPORT __attribute__((visibility("default")))
#else
#define DLLEXPORT __declspec(dllexport)
#endif

extern "C" {
DLLEXPORT bool initialize_audio_api(float metersPerUnit, std::shared_ptr<al::ISoundSystem> &outSoundSystem, std::string &errMsg)
{
	auto sys = std::shared_ptr<al::SoloudSoundSystem>(new al::SoloudSoundSystem {metersPerUnit}, [](al::SoloudSoundSystem *sys) {
		sys->OnRelease();
		delete sys;
	});
	if(sys->Initialize() != al::SoloudError::NoError)
		return false;
	outSoundSystem = sys;
	return true;
}
};
