//pesudo code

/*
void draw(short arr[FRAME_HEIGHT][FRAME_WIDTH]){//1: draw. 0: don't draw
    int penUp=1; //assume penUp first. when it is 0
    int scan_dir=1; //1 means left2right. -1 means right2left
    for (int r=0; r<FRAME_HEIGHT; r++){
        if (scan_dir==1){
       //find the first bit that is 1
       //case0: no such bit found, whole line empty, skip to next line 
       //case 0: bit is found 
            find the ending bit of that line 
            pendown and move, penup
        if that ending bit is not the end of line 
        find next bit 1
            if no more bit 1s are found-- no more lines to move until the end 
                skip to next line 
        }
    }

    
   
}


*/ 
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define FRAME_WIDTH 30  //x axis for the machine
#define FRAME_HEIGHT 8  //y axis for the machine 


//helper functions for testing only

void generateRandomBitArray(char bitArray[FRAME_HEIGHT][FRAME_WIDTH]) {
    // Seed the random number generator
    srand(time(NULL));

    for (int row = 0; row < FRAME_HEIGHT; row++) {
        // Generate a random number to determine the line type
        int lineType = rand() % 3;

        for (int col = 0; col < FRAME_WIDTH; col++) {
            switch (lineType) {
                case 0: // Empty line
                    bitArray[row][col] = 0;
                    break;
                case 1: // Fully filled line
                    bitArray[row][col] = 1;
                    break;
                case 2: // Scattered '1's
                    bitArray[row][col] = (rand() % 2) ? 1 : 0;
                    break;
                // case 3: // Chunks of '1's
                    if (col > 0 && bitArray[row][col - 1] == 1 && rand() % 2 == 0) {
                        bitArray[row][col] = 1; // Continue chunk
                    } else {
                        bitArray[row][col] = (rand() % 4 == 0) ? 1 : 0; // Start new chunk or empty
                    }
                    break;
            }
        }
    }
}


void visualizeBitArray(char bitArray[FRAME_HEIGHT][FRAME_WIDTH]) {
    // Print column numbers
    printf("    ");
    for (int col = 0; col < FRAME_WIDTH; col++) {
        printf("%02d ", col);
    }
    printf("\n");

    // Print the bit array with row numbers
    for (int row = 0; row < FRAME_HEIGHT; row++) {
        printf("%d | ", row); // Print row number
        for (int col = 0; col < FRAME_WIDTH; col++) {
            if (bitArray[row][col] == 1) {
                printf(" * "); // Print '*' for 1
            } else {
                printf("   "); // Print space for 0
            }
        }
        printf("\n"); // Move to the next line after each row
    }
}



//end of helper functions for testing


//return -1 if no such bit found
//value is either 0 or 1 
int find_next_start(char arr[FRAME_HEIGHT][FRAME_WIDTH], int row, int start_index, int dir){ 
    int position= start_index; 
    if (dir==1){
        for (;position<FRAME_WIDTH && arr[row][position]!=1; position++);  
                   //case 0, no such bit found, whole line empty
                   //then position will be FRAME_WIDTH
                   //case 1: bit is found. then  arr[row][position]==1
        if (position==FRAME_WIDTH){
            return -1; 
        }
        return position;
    }
    else if (dir==-1){
        for (;position>=0 && arr[row][position]!=1; position--);  
                   //case 0, no such bit found, whole line empty
                   //then position <0
        if (position<0){
            return -1; 
        }
        return position;
    }
    return -1;
}

int find_next_end(char arr[FRAME_HEIGHT][FRAME_WIDTH], int row, int start_index, int dir){ 
    int position= start_index; 
    if (dir==1){
        for (;position<FRAME_WIDTH && arr[row][position]!=0; position++);  
                //case 0, no such bit found, whole line full i.e.arr[row][FRAME_WIDTH-1]==1
                //then position will be FRAME_WIDTH, we should return FRAME_WIDTH-1
                //case 1: bit is found. then  arr[row][position]==0, we should return position-1
                position= (position < 0)? 0: position-1; //hopefully we never get position<0...
        return position;
    }
    else if (dir==-1){
        for (;position>=0 && arr[row][position]!=0; position--);  
                   //case 0, no such bit found, whole line full i.e.arr[row][0]==1
                   //but now position will be -1, we should return 0
                //case 1: bit is found. then  arr[row][position]==0, we should return position+1
                position= (position> FRAME_WIDTH-1)? FRAME_WIDTH-1: position+1; //hopefully we never get position>FRAME_WIDTH-1...
        return position;
    }
    return -1; //hopefully we never reach here 
}
//copied from main.c
struct point {
  int x; //correspond to FRAME_WIDTH
  int y; //correspond to FRAME_HEIGHT
float a; //angle
};

void penDown(){;}; 
void penUp(){;}; 
void drawLine(struct point newPos, struct point* actuatorPos, int* row){
    actuatorPos->x= newPos.x; 
    actuatorPos->y= newPos.y; 
     };  


int row; //for printing 


void draw_plane(char arr[FRAME_HEIGHT][FRAME_WIDTH], struct point* actualPos){//1: draw. 0: don't draw

    int scan_dir=1; //1 means left2right. -1 means right2left
    int start_pos=0; 
    int end_pos =0 ;
    for (int r=0; r<FRAME_HEIGHT; r++){

        if (scan_dir==1){
            start_pos=0; 
            end_pos =0 ;  //line is [start_pos, end_pos]   
        }
        else if (scan_dir==-1){
            start_pos=FRAME_WIDTH-1 ; 
            end_pos =FRAME_WIDTH-1  ;}  //line is [start_pos, end_pos]
        while ((scan_dir==1 && end_pos<FRAME_WIDTH-1)|| (scan_dir==-1 && end_pos>0)){
            //scan dir ==1
            //case 0, line [FRAME_WIDTH-1 ] is 0, then the start_pos will eventually be -1
            //case 1: line[FRAME_WIDTH-1] is 1, then the end_pos will be FRAME_WIDTH-1 eventually
            
            //scan dir ==-1 
            //case 0: line[0] is 0, then the start_pos will eventually be -1
            //case 1: line[0] is 1, then the end_pos will be 0 eventually 
            start_pos= find_next_start(arr, r, start_pos, scan_dir); 
            //printf("next start: %d", start_pos); 
            if (start_pos==-1){ //remaining line empty
                break; //move on to next line
            }
            //else find the end of the line segment
            end_pos= find_next_end(arr, r, start_pos, scan_dir); 
            struct point newPos= {start_pos, r,actualPos->a}; 
            drawLine(newPos, actualPos, &row);  //move to start pos 

            penDown(); //start drawing
            newPos.x = end_pos; 
            printf("from [%d, %d] to [%d, %d]\n", actualPos->y, actualPos->x, newPos.y, newPos.x); 
            drawLine(newPos, actualPos, &row);
            penUp(); 

            //end of segment
            if (scan_dir==1){
                 start_pos = end_pos+1; 
                if (start_pos>= FRAME_WIDTH){
                    break; 
                }
                }
            else if (scan_dir==-1){
                start_pos = end_pos-1; 
                if (start_pos< 0){
                    break; 
                }
            }
            }
            
           
        scan_dir *= -1; 
        //move to next line //FIXME: if next line is empty, no need to change dir
        //FIXME: changing line issue

    }
}
        
int main() {
    // char bitArray[FRAME_HEIGHT][FRAME_WIDTH] = {
    //     {0, 1, 0, 0, 1, 0, 1, 0},
    //     {1, 0, 1, 1, 0, 1, 0, 1},
    //     {0, 0, 0, 0, 0, 0, 0, 0},
    //     {1, 1, 1, 0, 1, 1, 1, 0},
    //     {0, 0, 1, 0, 0, 0, 1, 0},
    //     {1, 0, 1, 1, 1, 0, 1, 1},
    //     {0, 0, 0, 1, 0, 0, 0, 0},
    //     {1, 1, 1, 1, 1, 1, 1, 1}
    // };
    char bitArray[FRAME_HEIGHT][FRAME_WIDTH];
    generateRandomBitArray(bitArray);
    visualizeBitArray(bitArray);

    struct point actualPos= {0, 0, 0}; 
    draw_plane(bitArray, &actualPos); 
    return 0;
}
        
        
    

    
   
