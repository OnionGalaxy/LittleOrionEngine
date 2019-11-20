#pragma once

#include "Globals.h"
#include "imgui.h"
#include <vector>

struct ImGuiTextBuffer;

class EngineLog
{
public:
	enum class LogEntryType
	{
		LOG_INFO,
		LOG_SUCCESS,
		LOG_ERROR
	};

	enum class LogEntrySource
	{
		APP_LOG,
		OPENGL_LOG,
		ASSIMP_LOG
	};

	struct LogEntry 
	{
		LogEntry() = default;
		LogEntry(const LogEntrySource source, const LogEntryType type, const char *file, const int line, const char *message) :
			source(source),
			type(type),
			file(file),
			line(line),
			message(std::string(message))
		{}

		LogEntryType type = LogEntryType::LOG_ERROR;
		LogEntrySource source = LogEntrySource::APP_LOG;
		const char *file = nullptr;
		int line = 0;
		std::string message;
	};


	EngineLog();
	~EngineLog();

	void log(const LogEntrySource source, const LogEntryType type, const char* file, const int line, const char* message);
	void logFPS(const float fps);
	std::vector<float> getFPSData() const;

	void logMS(const float ms);
	std::vector<float> getMSData() const;
	
	void ShowConsoleWindow() const;

private:
	std::vector<LogEntry*> text_log;

	std::vector<float> fps_log;
	std::vector<float> ms_log;
};