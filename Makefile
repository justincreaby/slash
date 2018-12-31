# Makefile for slash autonomous car
#
# Copyright (C) 2018 Justin Creaby
#

race: race.cpp encoderFunctions.cpp readParameterAndPathFileToVector.cpp
	g++ -lroboticscape -o race encoderFunctions.cpp readParameterAndPathFileToVector.cpp race.cpp -I. -std=c++11

clean:
	rm -rf race *.o *~