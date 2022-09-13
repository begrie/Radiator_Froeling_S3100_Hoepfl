/******************************************************************************
 * Name:		FS_Filehelper.cpp
 * @brief    Datei-/Datenhandling im Flash-Dateisystem des ESP32
 * Created:	4/16/2020 3:59:16 PM
 * Author:	BG
 ******************************************************************************/

#include "FS_Filehelper.h"

bool FS_Filehelper::listDir(const char *dirname, uint8_t levels, bool with_content, fs::FS &fs)
{
  File root = fs.open(dirname);
  // File root = LittleFS.open(dirname);
  if (!root)
  {
    SERIALPRINT("FAILED TO OPEN DIRECTORY <<<  %s  >>>", dirname);
    return false;
  }

  if (!root.isDirectory())
  {
    SERIALPRINT("<<<  %s  >>> IS NOT A DIRECTORY", dirname);
    return false;
  }

  SERIALPRINT("\n### LISTING DIRECTORY    %s", root.path());

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      if (levels)
        listDir(file.path(), levels - 1, with_content, fs);
    }
    else // Dateiinfos ausgeben
    {
      Serial.printf("|-- %s (%d bytes) \n", file.path(), file.size());
      if (with_content)
      {
        while (file.available())
        {
          Serial.write(file.read());
        }
        Serial.println();
      }
    }

    file = root.openNextFile();
  }
  Serial.printf("### END listing DIRECTORY    %s \n", dirname);
  return true;
}

bool FS_Filehelper::readFile(const char *path, fs::FS &fs)
{
  DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t Read File <<<  %s  >>> ", path);

  File file = fs.open(path);
  // File file = LittleFS.open(path);
  if (!file || file.isDirectory())
  {
    SERIALPRINT("FAILED TO OPEN FILE <<<  %s  >>> FOR READING", path);
    return false;
  }

  // Serial.printf(">>>>>>>>>>>>>>> FILE= %s, SIZE= %d >>>>>>>>>>>>>>>\n", file.path(), file.size());
  // Serial.println(">>>>>>>>>>>>>>>>>>>>>>>>>> Inhalt: >>>>>>>>>>>>>>>>>>>>>>>>>>");
  SERIALPRINT("\n>>>>>>>>>>>>> FILE= %s, SIZE= %d >>>>>>>>>>>>>", file.path(), file.size());

  while (file.available())
  {
    Serial.write(file.read());
  }
  Serial.printf("\n<<<<<<<<<<<<<< End of file %s <<<<<<<<<<<<<<\n", file.path());
  return true;
}

bool FS_Filehelper::writeFile(const char *path, const char *message, fs::FS &fs)
{
  DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t Write file <<<  %s  >>> \n\t message: %s", path, message);

  File file = fs.open(path, FILE_WRITE);

  if (!file)
  {
    SERIALPRINT("FAILED TO OPEN FILE <<<  %s  >>> FOR WRITING", path);
    return false;
  }

  auto written_bytes = file.print(message);
  if (written_bytes)
  {
    DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t File <<<  %s  >>> written %d bytes with message\n\t%s", file.path(), written_bytes, message);
    return true;
  }
  else
  {
    DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t File <<<  %s  >>> WRITE FAILED with message\n\t%s", file.path(), message);
    return false;
  }
}

bool FS_Filehelper::appendFile(const char *path, const char *message, fs::FS &fs)
{
  DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t Append message % s to file: <<<  %s  >>> ", message, path);

  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    SERIALPRINT("FAILED TO OPEN FILE <<<  %s  >>> FOR APPEND \n\t message: %s", path, message);
    return false;
  }

  auto written_bytes = file.print(message);
  if (written_bytes)
  {
    DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t File <<<  %s  >>> appended %d bytes with message\n\t%s", file.path(), written_bytes, message);
    return true;
  }
  else
  {
    DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t File <<<  %s  >>> APPEND FAILED with message\n\t%s", file.path(), message);
    return false;
  }
}

bool FS_Filehelper::deleteFile(const char *path, fs::FS &fs)
{
  DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t Delete file <<<  %s  >>> ", path);

  if ((fs.open(path)).isDirectory())
  {
    SERIALPRINT("File <<<  %s  >>> is a directory -> NOTHING DELETED", path);
    return false;
  }

  if (fs.remove(path))
  {
    DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t File <<<  %s  >>> deleted", path);
    return true;
  }
  else
  {
    DEBUG_MESSAGE_FS_FILEHELPER_h("\n\t File <<<  %s  >>> DELETE FAILED", path);
    return false;
  }
}

bool FS_Filehelper::deleteDirectory(const char *path, fs::FS &fs)
{
  SERIALPRINT(" <<<  %s  >>> recursive with content", path);

  File root = fs.open(path);
  if (!root)
  {
    SERIALPRINT("FAILED TO OPEN DIRECTORY <<<  %s  >>> ", path);
    return false;
  }

  if (!root.isDirectory())
  {
    SERIALPRINT("<<<  %s  >>> IS NOT A DIRECTORY", path);
    return false;
  }

  File file = root.openNextFile();
  while (file)
  {
    if (file.isDirectory())
    {
      deleteDirectory(file.path(), fs);
      if (!fs.rmdir(file.path()))
      {
        SERIALPRINT("Delete of directory <<<  %s  >>> FAILED", file.path());
      }
      else
      {
        SERIALPRINT("Directory <<<  %s  >>> DELETED", path);
      }
    }
    else // Datei löschen -> NUR wenn im Verzeichnis
    {
      // diese Verrenkungen sind nötig, da root.openNextFile() ALLE SPIFFS-Dateien zurückliefert - unabhängig vom Pfad
      std::string _filename = file.path();
      std::size_t _found = _filename.find_last_of('/');
      std::string _pathname_only = _filename.substr(0, _found);
      std::string _filename_only = _filename.substr(_found);
      DEBUG_MESSAGE_FS_FILEHELPER_h("_filename =%s, _found=%d, _pathname_only=%s, _filename_only=%s",
                                    _filename.c_str(), _found, _pathname_only.c_str(), _filename_only.c_str());

      // ist leer falls root -> wieder ergänzen
      if (_pathname_only == "")
        _pathname_only = "/";

      DEBUG_MESSAGE_FS_FILEHELPER_h("\t>>>>>>>>>>>>> FILE= %s, SIZE= %d >>>>>>>>>>>>>\n", file.path(), file.size());
      if (_pathname_only.compare(path) == 0)
      { // löschen
        file.close();
        if (!fs.remove(_filename.c_str()))
        {
          SERIALPRINT("File <<<  %s  >>> DELETE FAILED", _filename.c_str());
          // return false;
        }
        else
        {
          SERIALPRINT("File <<<  %s  >>>  DELETED", _filename.c_str());
        }
      }
      else
      {
        SERIALPRINT("File <<<  %s  >>> is not in directory -> NOT DELETED", _filename.c_str());
      }
    }
    file = root.openNextFile();
  }
  SERIALPRINT("####### END deleting DIRECTORY  %s #######", path);
  return true;
}
