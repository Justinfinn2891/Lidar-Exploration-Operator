#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <random>
#include "raylib.h"



struct Robot{
    int x, y, w = 20, h = 40;
};




//Simple file reader for csv, may need modifcation based on new files
void OpenFile(std::vector<int>& data);

void CreateTextrue(std::vector<int> data, RenderTexture2D lidarTexture, Robot robot);

void CreateGrid(int x,int y,Robot robot);

void TestKey(Robot& robot, bool& flipped);

void DrawPoints(Robot robot,std::vector<int> data);


int main(){


    int screenWidth = 1920;
    int screenHeight = 1080;
    int test = 0;
    std::vector<int> data;
    bool flipped = false;
    srand(time(0));

    struct Robot robot = {screenWidth/2,screenHeight/2};

    OpenFile(data);

    InitWindow(screenWidth,screenHeight,"2d Mapping");
    SetTargetFPS(60);

    RenderTexture2D lidarTexture = LoadRenderTexture(screenWidth,screenHeight);
    CreateTextrue(data, lidarTexture, robot);

    //primary loop this is where the drawing happens
    while(!WindowShouldClose()){
        BeginDrawing();


            ClearBackground(BLACK);
            DrawFPS(10,10);



            CreateGrid(screenWidth,screenHeight, robot);

            DrawRectangle(robot.x-10,robot.y-20,robot.w,robot.h,PURPLE);

            TestKey(robot,flipped);


    
            DrawTextureRec(lidarTexture.texture,(Rectangle){0,0,(float)screenWidth, -(float)screenHeight},(Vector2){0,0},WHITE);


        EndDrawing();
    }


    CloseWindow();
    return 0;
}



void OpenFile(std::vector<int>& data){
    std::string line;
    int location , i = 0;

    std::ifstream file;
    file.open("tests/Random_coordinates.csv");
    if(file.is_open()){
        getline(file,line);
        while(line.length() != 4){
            location = line.find(',');
            data.push_back(stoi(line.substr(0,location)));
            line = line.substr(location+1, line.length());
            std::cout << line.length() << " " << data[i] << std::endl;
            i++;
        }
        data.push_back(stoi(line.substr(0,4)));
    }

    file.close();

    std::cout << data[0] << std::endl;
}


void CreateTextrue(std::vector<int> data, RenderTexture2D lidarTexture, Robot robot){
    BeginTextureMode(lidarTexture);
    for(int i = 0; i <= data.size()-1; i += 2){
        DrawCircle((robot.x + (data[i+1] * cos(data[i]))),(robot.y + (data[i+1] * sin(data[i]))),4,WHITE);
    }
    EndTextureMode();
}

void CreateGrid(int x,int y,Robot robot){
    for(int i = 100; i < x; i += 100){
        DrawCircleLines(robot.x,robot.y,i,LIGHTGRAY);
    }

}



void TestKey(Robot& robot, bool &flipped){
    if(IsKeyDown(KEY_UP)){
        robot.y += -5;
        if(flipped){
            int temp = robot.w;
            robot.w = robot.h;
            robot.h = temp;
            flipped = false;
        } 
    }
    if(IsKeyDown(KEY_DOWN)){
        robot.y += 5;
        if(flipped){
            int temp = robot.w;
            robot.w = robot.h;
            robot.h = temp;
            flipped = false;
        }  
    }
    if(IsKeyDown(KEY_LEFT)){
        robot.x += -5;
        if(!flipped){
        int temp = robot.w;
            robot.w = robot.h;
            robot.h = temp;
            flipped = true;
        }
    }
    if(IsKeyDown(KEY_RIGHT)){
        robot.x += 5;
        if(!flipped){
            int temp = robot.w;
            robot.w = robot.h;
            robot.h = temp;
            flipped = true;
        }
    }
}


