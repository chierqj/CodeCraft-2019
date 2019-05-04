#include "iostream"
#include "solve.h"
int main(int argc, char *argv[]) {
  std::cout << "Begin" << std::endl;

  if (argc < 6) {
    std::cout << "please input args: carPath, roadPath, crossPath, answerPath"
              << std::endl;
    exit(1);
  }

  std::string carPath(argv[1]);
  std::string roadPath(argv[2]);
  std::string crossPath(argv[3]);
  std::string presetAnswerPath(argv[4]);
  std::string answerPath(argv[5]);

  std::cout << "carPath is " << carPath << std::endl;
  std::cout << "roadPath is " << roadPath << std::endl;
  std::cout << "crossPath is " << crossPath << std::endl;
  std::cout << "presetAnswerPath is " << presetAnswerPath << std::endl;
  std::cout << "answerPath is " << answerPath << std::endl;

  // TODO:read input filebuf
  // TODO:process
  // TODO:write output file
  Solve solve(carPath, roadPath, crossPath, presetAnswerPath, answerPath);
  solve.run();

  return 0;
}