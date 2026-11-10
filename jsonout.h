/*
* jsonout.h -- JSON Lines status output for the DTE recorder
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#ifndef _JSONOUT_H
#define _JSONOUT_H

#include <sstream>
#include <string>

/** Turn JSON output on or off (off by default). */
void json_enable( bool enable );
bool json_enabled();

/** Builds one JSON object and writes it to stdout as a single line.

    JsonEvent( "status" ).str( "status", "REC" ).num( "frames", 10 ).send();

    send() takes a lock, writes the line, and flushes, so several threads can
    send events safely. Nothing is written when JSON output is off.
*/
class JsonEvent
{
public:
	explicit JsonEvent( const char *type );
	JsonEvent& str( const char *key, const std::string& value );
	JsonEvent& str( const char *key, const char *value );   // NULL -> null
	JsonEvent& num( const char *key, long long value );
	JsonEvent& dbl( const char *key, double value );
	JsonEvent& boolean( const char *key, bool value );
	JsonEvent& null( const char *key );
	void send();

	static std::string escape( const char *value );

private:
	void key( const char *name );
	std::ostringstream m_out;
};

#endif
