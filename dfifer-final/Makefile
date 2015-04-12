F=gfortran -g
CXX=g++ -g -std=c++11
CC=gcc -g
BUILD=build
TARGET=calib_server
SOURCE=$(wildcard *.cpp) $(wildcard minpack/*.f) $(wildcard triangulation/*.c)
OBJ=$(addsuffix .o, $(addprefix $(BUILD)/, $(basename $(notdir $(SOURCE))))) 
LDFLAGS= -lglut -lGL -lGLU -lglui -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_calib3d -lopencv_video -lopencv_features2d -pthread -lopencv_nonfree -lopencv_flann

all: $(TARGET)

$(BUILD)/%.o: minpack/%.f
	$(F) -c -o $@ $<

$(BUILD)/%.o: triangulation/%.c
	$(CC) -c -o $@ $< -DLIBRARY

$(BUILD)/%.o: %.cpp
	$(CXX) -c -o $@ $< -I triangulation

$(TARGET): $(OBJ)
	mkdir -p $(BUILD)
	$(CXX) -o $@ $^ $(LDFLAGS)

clean:
	rm $(BUILD)/*.o $(TARGET)

