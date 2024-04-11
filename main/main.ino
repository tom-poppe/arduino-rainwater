/* Copy secrets.h to secrets.local.h and fill in your values */
#pragma once

#if __has_include("secrets.local.h")
  #include "secrets.local.h"
#else
  #include "secrets.h"
#endif

/* Copy settings.h to settings.local.h and fill in your values */
#pragma once

#if __has_include("settings.local.h")
  #include "settings.local.h"
#else
  #include "settings.h"
#endif

void setup()
{
  
}

void loop()
{
  
}