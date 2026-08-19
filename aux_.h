
#ifndef AUX__H
#define AUX__H
#define sgn(x) ((x) > 0 ? 1 : ((x) < 0 ? -1 : 0))
#include <Arduino.h>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <vector>
#include <set>
#include <cctype>
using std::string;

namespace aux {
struct Command {
  char letter;
  int value;
  Command(char l, int v)
    : letter(l), value(v) {}
};

struct CommandList {
  std::vector<Command> commands;
  void addCommand(char letter, int value) {
    commands.push_back(Command(letter, value));
  }
  bool isEmpty() const {
    return commands.empty();
  }
  void print() const {
    for (const auto& cmd : commands) {
      std::cout << "Command: *" << cmd.letter << " Value: " << cmd.value << std::endl;
    }
  }
};

extern const std::set<char> VALID_COMMANDS;
extern char** commandArray;
string removeSubstring(const string& s, const string& sub);
CommandList parseCommands(const std::string& input);
char** commandsToStringArray(const std::string& input);
void freeStringArray(char** array);
char** parseCommandsToArray(const std::string& input);
int getCmdVal(const std::string& input);
string extractFileName(const char* path);

#define getFileName() aux::extractFileName(__FILE__)
#define getBuildInfo() (aux::extractFileName(__FILE__) + " (" + __DATE__ + " " + __TIME__ + ")")
}

extern int resetTime;
extern int resetTimeMax;
extern int checkTime;
extern int checkTimeMax;

void buildMessage(String& output, const char* name, int value, bool isOk, bool pb = true);
bool checkTimer(int tc, int tmax);
double sigmoid(double x, double k);
#endif