CC=g++
CFLAGS=
LDFLAGS=-lopencv_core -lopencv_imgproc -lopencv_highgui

yuv422_img: main.cpp yuv422_img.cpp
	$(CC) -o yuv422_display main.cpp yuv422_img.cpp $(CFLAGS) $(LDFLAGS)

clean:
	rm -f yuv422_display
	
	
