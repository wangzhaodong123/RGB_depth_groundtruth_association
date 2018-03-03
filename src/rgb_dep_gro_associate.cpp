/*
 * 将某一时刻的位置信息，深度图像，彩色图像进行对齐的程序
 * 输入：rgb.txt depth.txt groundtruth.txt
 * 输出：association.txt
 * 
 * rgb时间与depth的时间相差0.02以内视为匹配  MAX_DIFFERENCE_RGB_DEPTH
 * rgb时间与groundtruth的时间相差0.01以内视为匹配  MAX_DIFFERENCE_RGB_GROUNDTRUTH
 * 
 */
#include <iostream>
#include <fstream>
#include <ostream>
#include <boost/format.hpp>
#include <boost/concept_check.hpp>

using namespace std;

//表示位姿的结构体
struct pose
{
  double tx;
  double ty;
  double tz;
  double qx;
  double qy;
  double qz;
  double qw;
};

//定义匹配阀值
#define MAX_DIFFERENCE_RGB_DEPTH 0.02
#define MAX_DIFFERENCE_RGB_GROUNDTRUTH 0.01

int main( int argc, char** argv )
{
  //校验输入格式
  if ( argc != 5 )
  {
    cerr<<" 输入格式： ./rgb_dep_gro_associate rgb.txt depth.txt groundtruth.txt association.txt "<<endl;
    return 1;    
  }
  
  //读取txt文件时的一些临时变量
  string string_unused;
  string string_tem;
  double time_tem;
  
  //打开rgb.txt文件
  ifstream file_rgb( argv[1] );
  if ( !file_rgb.is_open() )
  {
    cerr<<" rgb.txt文件打开失败！ "<<endl;
    return 1;
  }
  //建立存储时间和地址的容器
  vector< double > time_rgb;
  vector< string > address_rgb;
  //跳过前三行信息
  getline( file_rgb, string_unused, '\n' );
  getline( file_rgb, string_unused, '\n' );
  getline( file_rgb, string_unused, '\n' );
  //逐行将数据读入容器
  while( !file_rgb.eof() )
  {
    //读入时间，高位不计
    file_rgb>>time_tem;
    time_rgb.push_back(time_tem-1311877000);
    //读入地址
    file_rgb>>string_tem;
    address_rgb.push_back(string_tem);
  }
  file_rgb.close();

  //打开depth.txt文件
  ifstream file_depth( argv[2] );
  if ( !file_depth )
  {
    cerr<<" depth.txt文件打开失败！ "<<endl;
    return 1;
  }
  //建立存储时间和地址的容器
  vector< float > time_depth;
  vector< string > address_depth;
  //跳过前三行信息
  getline( file_depth, string_unused, '\n' );
  getline( file_depth, string_unused, '\n' );
  getline( file_depth, string_unused, '\n' );
  //逐行将数据读入容器
  while( !file_depth.eof() )
  {
    //读入时间，高位不计
    file_depth>>time_tem;
    time_depth.push_back(time_tem-1311877000);
    //读入地址
    file_depth>>string_tem;
    address_depth.push_back(string_tem);
  }
  file_depth.close();
  
  //打开groundtruth.txt文件
  ifstream file_groundtruth( argv[3] );
  if ( !file_groundtruth.is_open() )
  {
    cerr<<" groundtruth.txt文件打开失败！ "<<endl;
    return 1;
  }
  //建立存储时间和位置的容器
  vector< double > time_groundtruth;
  vector< pose > pose_groundtruth;
  pose pose_tem;
  //跳过前三行信息
  getline( file_groundtruth, string_unused, '\n' );
  getline( file_groundtruth, string_unused, '\n' );
  getline( file_groundtruth, string_unused, '\n' );
  //逐行将数据读入容器
  while( !file_groundtruth.eof() )
  {
    //读入时间，高位不计
    file_groundtruth>>time_tem;
    time_groundtruth.push_back(time_tem-1311877000);
    //读入位置
    file_groundtruth>>pose_tem.tx>>pose_tem.ty>>pose_tem.tz>>pose_tem.qx>>pose_tem.qy>>pose_tem.qz>>pose_tem.qw;
    pose_groundtruth.push_back(pose_tem);
  }
  file_groundtruth.close();
  
  //打开association.txt文件，有则清空，无则新建
  ofstream file_association( argv[4] );
  if ( !file_association.is_open() )
  {
     cerr<<" association.txt文件打开失败！ "<<endl;
     return 1;
  }
  //写入文件头提示信息
  file_association<<"# association of rgb,depth,groundtruth "<<endl;
  file_association<<"# rgb depth tx ty tz qx qy qz qw "<<endl;

  //循环开始
  int count_rgb = 0;
  int count_depth = 0;
  int count_groundtruth = 0;
  int count_match = 0;
  for (; count_rgb<time_rgb.size(); count_rgb++ )
  {
    double time_distance=1000;
    int count_tem;

    //在rgb对应的depth前后50个范围内查找时间距离最小值
    for ( int i=-50; i<51; i++ )
    {
      count_tem = count_rgb+i;
      if ( count_tem<0 || count_tem>time_depth.size() )
	continue;
      double tem = abs(time_rgb[count_rgb] - time_depth[count_tem]) ;
      if ( tem < time_distance)
      {
	time_distance = tem;
	count_depth = count_tem;
      }
    }
    //如果rgb和depth没有匹配到，则不在匹配groundtruth
    if (time_distance>MAX_DIFFERENCE_RGB_DEPTH)
      continue;
    
    //在groundtruth里寻找最佳匹配值
    //从头开始寻找，找到一个合理范围内的之后，再向后查询50个值，找到这二十个当中的最小值
    double tem;
    bool flag=false;
    for ( int i=0; i<time_groundtruth.size(); i++ )
    {
      tem = abs( time_rgb[count_rgb] - time_groundtruth[i] );
      if ( tem <MAX_DIFFERENCE_RGB_GROUNDTRUTH )
      {
	time_distance = tem;
	count_groundtruth =i;
	for ( int j=1; j<50; j++ )
	{
	  if (i+j>time_groundtruth.size())
	    break;
	  double tem_1 = abs( time_rgb[count_rgb] - time_groundtruth[i+j] );
	  if ( tem_1 < time_distance )
	  {
	    count_groundtruth = i+j;
	    time_distance = tem_1;
	  }
	}
	flag=true;
	break;
      }
    }
    
    //如果没有对应的groundtruth值，则进入下一个循环
    if( !flag )
      continue;

    //成功匹配到一组数据，计数位加一
    count_match++;
    //向association.txt中写入数据
    file_association<<address_rgb[count_rgb]<<' '
		    <<address_depth[count_depth]<<' '
		    <<pose_groundtruth[count_groundtruth].tx<<' '
		    <<pose_groundtruth[count_groundtruth].ty<<' '
		    <<pose_groundtruth[count_groundtruth].tz<<' '
		    <<pose_groundtruth[count_groundtruth].qx<<' '
		    <<pose_groundtruth[count_groundtruth].qy<<' '
		    <<pose_groundtruth[count_groundtruth].qz<<' '
		    <<pose_groundtruth[count_groundtruth].qw<<' '<<endl;
  }
  
  file_association.close();
  cout<<" rgb和depth成功匹配到 "<<count_match<<"对"<<endl;
  
  return 0;
}