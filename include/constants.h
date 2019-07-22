#ifndef CONSTANTS_H_
#define CONSTANTS_H_

/**
 * Constants is a singleton class that reads constants from a file and stores them in a set of public member
 * variables to be used later in the code.
 *
 * Names and default values for these variables are read from constant_declarations.h. This file is used so
 * that values can be quickly and easily changed and tested without having to recompile code each time.
 *
 * Sample Usage:
 * Constants* constants = Constants::GetInstance();
 * printf("Test Int %d\n", constants->testinteger);
 *
 * This would create or retrieve an instance of this class with all the constants initialized.
 */
class Constants {
 public:
  // List the definitions from constant_declarations.h as public members of this class.
#define DECLARE(name, default_value) \
double name;
#include "constant_declarations.h"
#undef DECLARE


  /**
   * Returns a singleton instance of this class to ensure that users don't create multiple classes with
   * references to variables that should have a single constant value.
   *
   * To be used in place of constructor for this class.
   */
  static Constants* GetInstance();

 private:
  Constants();

  // A static reference to this class to ensure that only one instance of it is created.
  static Constants* instance_;
};

#endif
