#ifndef _ModuleFileSystem_H
#define _ModuleFileSystem_H

#include <Module/Module.h>
#include <string>
#include <vector>
#include <memory>

#include <physfs/physfs.h>

class ModuleFileSystem : public Module
{

public:
	enum class FileType
	{
		MODEL,
		TEXTURE,
		DIRECTORY,
		ARCHIVE,
		UNKNOWN
	};
	struct File {
		File() = default;
		File(const std::string & path, const std::string & name);
		File(const std::string & path);
		std::string filename;
		std::string file_path;
		std::string filename_no_extension;
		ModuleFileSystem::FileType file_type;


		std::vector<std::shared_ptr<File>> children;
		std::shared_ptr<File> parent;
		size_t sub_folders = 0;
		bool operator==(const File& compare);
	private:
		void FillChilds();
	};

	bool Init() override;
	bool CleanUp() override;
	ModuleFileSystem() = default;
	~ModuleFileSystem();

	char* Load( const char* file_name, size_t & size) const;
	unsigned int Save(const char* file_name, const void* buffer, unsigned int size, bool append = false) const;

	bool Remove(const File & file);
	bool Exists(const char* file) const;
	std::string MakeDirectory(const std::string & new_directory_full_path);
	bool Copy(const char* source, const char* destination);
	
	FileType GetFileType(const char *file_path, const PHYSFS_FileType & file_type = PHYSFS_FileType::PHYSFS_FILETYPE_OTHER) const;
	void GetAllFilesInPath(const std::string & path, std::vector<std::shared_ptr<File>> & files, bool directories_only = false) const;
	size_t GetNumberOfFileSubFolders(const std::shared_ptr<ModuleFileSystem::File> & file) const;

	void RefreshFilesHierarchy();
private:
	char *save_path = NULL;
	bool IsValidFileName(const char * file_name) const;
	std::string GetFileExtension(const char *file_path) const;

public:

	std::shared_ptr<File> root_file;
};


#endif // !_ModuleFileSystem_H
