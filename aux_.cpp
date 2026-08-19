
#include "aux_.h"

namespace aux {
const std::set<char> VALID_COMMANDS = {
  'A',
  'F',
  'D',
  'I',
  'P',
  'R',
  'Q',
  'T',
};

char** commandArray = parseCommandsToArray("This is a test *H200*G50*A100");

string removeSubstring(const string& s, const string& sub) {
  if (sub.empty()) return s;
  string out = s;
  size_t pos = 0;
  while ((pos = out.find(sub, pos)) != string::npos) {
    out.replace(pos, sub.length(), "");
  }
  return out;
}

CommandList parseCommands(const std::string& input) {
  CommandList result;
  for (size_t i = 0; i < input.length(); i++) {
    if (input[i] == '*' && i + 1 < input.length()) {
      char letter = input[i + 1];
      if (VALID_COMMANDS.find(letter) != VALID_COMMANDS.end()) {
        std::string numStr = "";
        size_t j = i + 2;
        while (j < input.length() && isdigit(input[j])) {
          numStr += input[j];
          j++;
        }
        if (!numStr.empty()) {
          int value = std::stoi(numStr);
          result.addCommand(letter, value);
          i = j - 1;
        }
      }
    }
  }
  return result;
}

char** commandsToStringArray(const std::string& input) {
  CommandList cmdList = parseCommands(input);
  char** array = new char*[cmdList.commands.size() + 1];
  for (size_t i = 0; i < cmdList.commands.size(); i++) {
    std::string cmdStr = "*" + std::string(1, cmdList.commands[i].letter) + std::to_string(cmdList.commands[i].value);
    array[i] = new char[cmdStr.length() + 1];
    strcpy(array[i], cmdStr.c_str());
  }
  array[cmdList.commands.size()] = nullptr;
  return array;
}

void freeStringArray(char** array) {
  if (array == nullptr) return;
  for (int i = 0; array[i] != nullptr; i++) {
    delete[] array[i];
  }
  delete[] array;
}

char** parseCommandsToArray(const std::string& input) {
  std::vector<std::string> commandStrings;
  for (size_t i = 0; i < input.length(); i++) {
    if (input[i] == '*' && i + 1 < input.length()) {
      char letter = input[i + 1];
      if (VALID_COMMANDS.find(letter) != VALID_COMMANDS.end()) {
        std::string numStr = "";
        size_t j = i + 2;
        while (j < input.length() && isdigit(input[j])) {
          numStr += input[j];
          j++;
        }
        if (!numStr.empty()) {
          std::string cmdStr = "*" + std::string(1, letter) + numStr;
          commandStrings.push_back(cmdStr);
          i = j - 1;
        }
      }
    }
  }
  char** array = new char*[commandStrings.size() + 1];
  for (size_t i = 0; i < commandStrings.size(); i++) {
    array[i] = new char[commandStrings[i].length() + 1];
    strcpy(array[i], commandStrings[i].c_str());
  }
  array[commandStrings.size()] = nullptr;
  return array;
}

int getCmdVal(const std::string& input) {
  CommandList commands = parseCommands(input);
  return commands.isEmpty() ? -1 : commands.commands[0].value;
}

string extractFileName(const char* path) {
  const char* file = strrchr(path, '\\');
  if (!file) file = strrchr(path, '/');
  return (file) ? string(file + 1) : string(path);
}
}

int resetTime = millis();
int resetTimeMax = 700 * 1000;

int checkTime = millis();
int checkTimeMax = 6 * 3600 * 1000;

int pulseTime = millis();
int pulseTimeMax = 60 * 1000;


void buildMessage(String& output, const char* name, int value, bool isOk,bool pb) {
  char buffer[64];
  const char* alarmStatus = (!pb) ? "alarmNA" : (isOk ? "alarmOFF" : "alarmON");
  snprintf(buffer, sizeof(buffer), "%s %d %s", name, value, alarmStatus);
  output = buffer;
}


bool checkTimer(int tc, int tmax) {
  return ((millis() - tc) <= tmax);
}

double sigmoid(double x,double k) {
    return 1.0 / (1.0 + exp(-k*(x-0.5)));
}