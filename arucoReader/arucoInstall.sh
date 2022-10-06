opencvVersion=$(pkg-config --modversion opencv)

if [ "$opencvVersion" != "3.4.16" ]; then
  chmod +x ../opencv3.4.16Install.sh
fi
cwd=$(pwd)
cd ~ || exit
git clone https://github.com/nlohmann/json.git
cd json || exit
mkdir build
cd build || exit
cmake ..
sudo make -j"$(nproc)" install
cd "$cwd" || exit
mkdir build
cd build || exit
cmake ..
make -j
