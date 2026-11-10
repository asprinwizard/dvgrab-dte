/*
* jsonout.cc -- JSON Lines status output for the DTE recorder
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*/

#include "jsonout.h"

#include <pthread.h>
#include <stdio.h>
#include <iomanip>

static bool g_json = false;
static pthread_mutex_t g_json_mutex = PTHREAD_MUTEX_INITIALIZER;

void json_enable( bool enable )
{
	g_json = enable;
}

bool json_enabled()
{
	return g_json;
}

JsonEvent::JsonEvent( const char *type )
{
	m_out << "{\"type\":" << escape( type );
}

void JsonEvent::key( const char *name )
{
	m_out << ',' << escape( name ) << ':';
}

JsonEvent& JsonEvent::str( const char *name, const std::string& value )
{
	// c_str(): some dvgrab file names carry a trailing NUL from "<< ends".
	return str( name, value.c_str() );
}

JsonEvent& JsonEvent::str( const char *name, const char *value )
{
	key( name );
	if ( value )
		m_out << escape( value );
	else
		m_out << "null";
	return *this;
}

JsonEvent& JsonEvent::num( const char *name, long long value )
{
	key( name );
	m_out << value;
	return *this;
}

JsonEvent& JsonEvent::dbl( const char *name, double value )
{
	key( name );
	m_out << std::fixed << std::setprecision( 2 ) << value;
	return *this;
}

JsonEvent& JsonEvent::boolean( const char *name, bool value )
{
	key( name );
	m_out << ( value ? "true" : "false" );
	return *this;
}

JsonEvent& JsonEvent::null( const char *name )
{
	key( name );
	m_out << "null";
	return *this;
}

void JsonEvent::send()
{
	if ( !g_json )
		return;
	m_out << "}\n";
	std::string line = m_out.str();
	pthread_mutex_lock( &g_json_mutex );
	fputs( line.c_str(), stdout );
	fflush( stdout );
	pthread_mutex_unlock( &g_json_mutex );
}

std::string JsonEvent::escape( const char *value )
{
	std::string out = "\"";
	for ( const unsigned char *p = ( const unsigned char * ) value; *p; ++p )
	{
		switch ( *p )
		{
		case '"': out += "\\\""; break;
		case '\\': out += "\\\\"; break;
		case '\n': out += "\\n"; break;
		case '\r': out += "\\r"; break;
		case '\t': out += "\\t"; break;
		default:
			if ( *p < 0x20 )
			{
				char buf[ 8 ];
				snprintf( buf, sizeof( buf ), "\\u%04x", *p );
				out += buf;
			}
			else
				out += ( char ) *p;
		}
	}
	out += "\"";
	return out;
}
