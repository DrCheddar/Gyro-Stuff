#include "constants.h"

#include <cstddef>

Constants* Constants::instance_ = NULL;

Constants* Constants::GetInstance() {
	if (instance_ == NULL) {
		instance_ = new Constants();
	}
	return instance_;
}

Constants::Constants () {
    #define DECLARE(name, default_value) name = default_value;
	#include "constant_declarations.h"
	#undef DECLARE
}
