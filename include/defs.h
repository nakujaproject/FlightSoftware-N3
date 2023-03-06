#ifndef DEFS_H
#define DEFS_H

/* debug enable for use during testing */
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#define debugf(x, y) Serial.printf(x, y)

#else

#define debug(x)
#define debugln(x)
#define debugf(x, y)

#endif

/* timing constant */
#define SETUP_DELAY 300

/* flight constants */
#define EJECTION_HEIGHT 1000 // eject at 1000m AGL


#endif