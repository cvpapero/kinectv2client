#pragma once
#include "msgpack.hpp"
#include <iostream>
namespace MsgPackUtil{
	template <typename T>
	inline void unpack( const std::string message, T &v )
	{
		try
		{
			msgpack::sbuffer sbuf;
			msgpack::unpacked msg;
			msgpack::unpack( &msg, reinterpret_cast<const char*>( message.data()), message.size() );
			msg.get().convert( &v );
		}
		catch( const std::exception &e )
		{
			std::cout << "exception:" << e.what() << std::endl;
		}
	}
}