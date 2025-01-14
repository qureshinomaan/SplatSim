#!/bin/bash -ex
NCORES=$(($(grep -c ^processor /proc/cpuinfo) + 1))
wget -c 'https://sourceforge.net/projects/boost/files/boost/1.68.0/boost_1_68_0.tar.bz2/download'
tar xf download
cd boost_1_68_0
./bootstrap.sh --with-libraries=system,thread,program_options
./b2 -j${NCORES} cxxflags="-fPIC" runtime-link=static variant=release link=static install
cd ..
rm -rf download.bz2 boost_1_68_0

