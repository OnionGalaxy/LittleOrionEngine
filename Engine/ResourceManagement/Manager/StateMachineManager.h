#ifndef _STATEMACHINEMANAGER_H_
#define _STATEMACHINEMANAGER_H_

#include "Filesystem/File.h"

#include <memory>
#include <string>

class StateMachine;
class Metafile;
class Path;

class StateMachineManager

{
public:
	StateMachineManager() = default;
	~StateMachineManager() = default;

	static FileData Binarize(const StateMachine& state_machine);
	static std::shared_ptr<StateMachine> Load(Metafile* metafile, const FileData& resource_data);
	static FileData Create();
};

#endif // !_H_STATEMACHINEMANAGER_

