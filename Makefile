OPTFLAGS := -O3 -ffast-math  -I /usr/local/include/eigen3 -lnlopt -lm 
WHENWORKS := -DNDEBUG 

OUTPUT_DIR := outputs

OBJS := $(OUTPUT_DIR)/Global.o $(OUTPUT_DIR)/Human.o $(OUTPUT_DIR)/Robot.o $(OUTPUT_DIR)/Print.o $(OUTPUT_DIR)/State.o $(OUTPUT_DIR)/Planning.o $(OUTPUT_DIR)/Estimation.o $(OUTPUT_DIR)/DG.o $(OUTPUT_DIR)/main.o


all: $(OUTPUT_DIR) $(OBJS)
	@echo "Combining Everything..."
	@g++ -Wall -std=c++11 $(OPTFLAGS) $(OBJS) -o out
	@echo "Done ✔"

$(OUTPUT_DIR)/State.o: State.cpp State.h Global.h | $(OUTPUT_DIR)
	@echo "Compiling State..."
	@g++ -c $(OPTFLAGS) State.cpp -o $(OUTPUT_DIR)/State.o

$(OUTPUT_DIR)/Planning.o: Planning.cpp Planning.h State.h Global.h | $(OUTPUT_DIR)
	@echo "Compiling Planning..."
	@g++ -c $(OPTFLAGS) Planning.cpp -o $(OUTPUT_DIR)/Planning.o

$(OUTPUT_DIR)/DG.o: DG.cpp DG.h Planning.h State.h Global.h | $(OUTPUT_DIR)
	@echo "Compiling DG..."
	@g++ -c $(OPTFLAGS) DG.cpp -o $(OUTPUT_DIR)/DG.o

$(OUTPUT_DIR)/main.o: main.cpp State.h Global.h Planning.h | $(OUTPUT_DIR)
	@echo "Compiling Main..."
	@g++ -c $(OPTFLAGS) main.cpp -o $(OUTPUT_DIR)/main.o

$(OUTPUT_DIR)/Global.o: Global.cpp Global.h | $(OUTPUT_DIR)
	@echo "Compiling Global..."
	@g++ -c $(OPTFLAGS) Global.cpp -o $(OUTPUT_DIR)/Global.o

$(OUTPUT_DIR)/Robot.o: Robot.cpp Robot.h | $(OUTPUT_DIR)
	@echo "Compiling Robot..."
	@g++ -c $(OPTFLAGS) Robot.cpp -o $(OUTPUT_DIR)/Robot.o

$(OUTPUT_DIR)/Human.o: Human.cpp Human.h | $(OUTPUT_DIR)
	@echo "Compiling Human..."
	@g++ -c $(OPTFLAGS) Human.cpp -o $(OUTPUT_DIR)/Human.o
	
$(OUTPUT_DIR)/Print.o: Print.cpp Print.h | $(OUTPUT_DIR)
	@echo "Compiling Print..."
	@g++ -c $(OPTFLAGS) Print.cpp -o $(OUTPUT_DIR)/Print.o

$(OUTPUT_DIR)/Estimation.o:	Estimation.cpp Estimation.h State.h Global.h | $(OUTPUT_DIR)
	@echo "Compiling Estimation..."
	@g++ -c $(OPTFLAGS) Estimation.cpp -o $(OUTPUT_DIR)/Estimation.o

clean:
	@echo "Removing all files in outputs..."
	@find $(OUTPUT_DIR) -type f -delete
	@echo "Done ✔"

run:
	@./out
nlopt:nlopt.cpp|
	g++ nlopt.cpp -o out -lnlopt -lm
	./out