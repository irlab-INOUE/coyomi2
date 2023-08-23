# インストール先を変更したい場合は，ROOT のパスを修正する
ROOT = ../../
INC_DIR = include
LIB_DIR = lib

all: install
	cd sample;mkdir build;cd build;cmake ..;make

clean:
	$(RM) *~ *.o;
	cd sample; rm -rf build

install:
	cp matrix.h $(ROOT)/$(INC_DIR)/matrix.h
	#cp sparsematrix.h $(HOME)/include/sparsematrix.h

uninstall:
	rm $(ROOT)/$(INC_DIR)/matrix.h
	#rm $(HOME)/include/sparsematrix.h


