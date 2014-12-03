#include "Util.h"
#include "base64.h"
using namespace std;
using namespace cv;
namespace Util{
  /* FUNCTION : toString
   * 様々な型からstring型に変換する
   *
   * Param : int : n : 文字列に変換したい数値(int型)
   * Return : std::string : 変換後の文字列
   */
  std::string toString(int n){std::stringstream ss; ss << n; return ss.str();}
  /* FUNCTION : replaceAll
   * 渡された文字列に対してreplace(置換)を可能な限り行う
   * "aaaabaaaababaab"に対してすべてのbをcに置換するとする
   * その結果"aaaacaaaacacaac"となる
   *
   * Param : std::string & : str :文字列操作を加える文字列
   * : const std::string : from :変更を加えたい文字
   * : const std::string : to :変更後の文字
   * Return : void : なし
   */
  void replaceAll(std::string &str,const std::string from, const std:: string to){
    std::string::size_type pos = 0;
    while((pos=str.find(from))!= std::string::npos)
      {
	str.replace(pos,from.size(),to);
      }
  }
  /* FUNCTION : split
   * 渡された文字列に対して指定文字列でsplit(分割)を可能な限り行う
   * Param : sstring : s :文字列操作を加える文字列(元の文字列)
   * Return : vector : 分割された文字列が入っている動的配列
   */
  std::vector<std::string> split(std::string s, std::string t) {
    std::vector<std::string> v;
    for (int p = 0; (p = s.find(t)) != s.npos; )
      {
	v.push_back(s.substr(0, p));
	s = s.substr(p + t.size());
      }
    v.push_back(s);
    return v;
  }
  std::vector<std::string> splitFirst(std::string s, const std::string t) {
    std::vector<std::string> v;
    int p = s.find(t);
    if (p != s.npos)
      {
	v.push_back(s.substr(0, p));
	s = s.substr(p + t.size());
      }
    v.push_back(s);
    return v;
  }
  cv::Mat convertVec2Mat( const std::vector<unsigned char>& vec ) {
    return cv::imdecode( cv::Mat( vec ), CV_LOAD_IMAGE_UNCHANGED );
  }
  cv::Mat convertString2Mat( std::string str) {
    std::vector< unsigned char > buf( str.begin( ), str.end( ) );
    buf.push_back('0');
    return cv::imdecode( cv::Mat( buf ), CV_LOAD_IMAGE_UNCHANGED );
  }
  std::string convertMat2String( cv::Mat mat ) {
    if( mat.data == NULL ) {
      return "";
    }
    std::vector< unsigned char > buf;
    std::vector< int > param = std::vector< int > ( 2 ) ;
    param[ 0 ] = CV_IMWRITE_JPEG_QUALITY;
    param[ 1 ] = 95;
    cv::imencode( ".jpg", mat, buf, param );
    std::istringstream iss(string( buf.begin(), buf.end() ) );
    std::ostringstream oss;
    base64::Encode( iss, oss );
    return oss.str();	
  }
  /*
    std::string GetCurrentTimeString(){
    time_t timer;
    struct tm local;
    timer = time( NULL );
    localtime_s( &local, &timer );
    char strDate[128];
    sprintf_s( strDate, "%4d-%02d-%02d-%02d-%02d-%02d",
    local.tm_year + 1900,
    local.tm_mon + 1,
    local.tm_mday,
    local.tm_hour,
    local.tm_min,
    local.tm_sec
    );
    return std::string( strDate );
    }
  */
} 
