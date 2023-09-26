#include <filesystem>
#include <string>

// File System Utilities. These are helper functions that sit ontop of the Filesystem library included since C++ 17.
// These functions try to make file handling with C++ easier, and implement many functions used everyday in apps that rely on much file-processing.

// Folder basics
bool CreateNewFolder(std::filesystem::path path);
bool DeleteFolder(std::filesystem::path path);
bool RenameFolder(std::filesystem::path path, std::string newName);

// File basics
bool DeleteFile(std::filesystem::path);
bool RenameFile(std::filesystem::path, std::string newName);

//File IO
bool WriteTextFile(std::filesystem::path, std::string text);
bool WriteBinaryFile(std::filesystem::path, char* bytes);
char* ReadBinaryFile(std::filesystem::path);
std::string ReadTextFile(std::filesystem::path);

// File/Folder discovery
std::vector<std::filesystem::path> GetFilesWithExtension(std::filesystem::path, std::string extension);
std::vector<std::filesystem::path> GetFilesWithName(std::filesystem::path, std::string filenameContains);
std::vector<std::filesystem::path> GetFoldersWithName(std::filesystem::path, std::string folderContains);
std::vector<std::filesystem::path> SortPathsByNumericValue(std::vector<std::filesystem::path> paths, bool ascending);

// Path conversion
std::string GetFilenameFromPath(std::filesystem::path path);
std::string GetFileExtension(std::filesystem::path);
std::filesystem::path GetParentFolder(std::filesystem::path);
int GetNumericValueFromFilename(std::filesystem::path);

