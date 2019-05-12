uhubctl -a off -p 2
git commit -a --allow-empty-message -m ""
git push -u origin master
g++ off.cpp -o off -lwiringPi -O2 -W
uhubctl -a on -p 2
sleep 1
make -j4 && ./main
sleep 1
./off
