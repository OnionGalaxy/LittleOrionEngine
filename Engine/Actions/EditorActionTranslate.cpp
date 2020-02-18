#include "EditorActionTranslate.h"



EditorActionTranslate::EditorActionTranslate(float3 &previous, float3 &current, GameObject* go)
{
	
	previous_position = previous;
	current_position = current;
	action_GO = go;
	UUID_go = go->UUID;
	
}


EditorActionTranslate::~EditorActionTranslate()
{
}


void EditorActionTranslate::Undo()
{
	action_GO->transform.SetTranslation(previous_position);
	return;
}

void EditorActionTranslate::Redo()
{
	action_GO->transform.SetTranslation(current_position);
	return;
}

void EditorActionTranslate::CheckGO(GameObject * go)
{
	if(go->UUID == UUID_go)
	{
		action_GO = go;
	}
}
