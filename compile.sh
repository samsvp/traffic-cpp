if [ "$(g++ --version | grep g++ | rev | cut -d' ' -f1 | rev | cut -d'.' -f1)" -gt "10" ]; then
    CC="g++"
else
    CC="g++-13"
fi
$CC -std=c++20 main.cpp -lraylib -lglfw -lGL -lm -lpthread -ldl -lrt -lX11 -ltbb
