#ifndef _PANELPROJECTEXPLORER_H_
#define _PANELPROJECTEXPLORER_H_

#include "EditorUI/Panel/Panel.h"
#include "Module/ModuleFileSystem.h"

class Metafile;

class PanelProjectExplorer : public Panel
{

public:
	PanelProjectExplorer();
	~PanelProjectExplorer() = default;

	void Render() override;

	void ShowFoldersHierarchy(const Path& file);

	void ProcessMouseInput(Path* file);
	void ProcessResourceMouseInput(Path* file);
	void ShowFilesInExplorer();

	void ShowFileSystemActionsMenu(Path* path);

	void FilesDrop() const;
	void ResourceDragSource(Metafile* file) const;

private:
	void InitResourceExplorerDockspace();

	void ShowMetafileIcon(Path* file);

private:
	float file_size_width = 100.f;
	float file_size_height = 150.f;

	Path* selected_folder = nullptr;
	Path* selected_file = nullptr;

	ImGuiID project_explorer_dockspace_id;
};
#endif //_PANELPROJECTEXPLORER_H_