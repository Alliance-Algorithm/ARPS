#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <time.h>
// [日志实现]>
class Logger
{
public:
    enum log_level
    {
        debug,
        info,
        warning,
        error
    }; // 日志等级
    enum log_target
    {
        file,
        terminal,
        file_and_terminal
    }; // 日志输出目标
public:
    Logger();
    Logger(log_target target, log_level level, const std::string &path);
    ~Logger();

    void DEBUG(const std::string &text);
    void INFO(const std::string &text);
    void WARNING(const std::string &text);
    void ERRORS(const std::string &text);

private:
    std::ofstream m_outfile;                                   // 将日志输出到文件的流对象
    log_target m_target;                                       // 日志输出目标
    std::string m_path;                                        // 日志文件路径
    log_level m_level;                                         // 日志等级
    void output(const std::string &text, log_level act_level); // 输出行为
};
std::string currTime()
{
    // 获取当前时间，并规范表示
    char tmp[64];
    time_t ptime;
    time(&ptime); // time_t time (time_t* timer);
    strftime(tmp, sizeof(tmp), "%Y-%m-%d %H:%M:%S", localtime(&ptime));
    return tmp;
}
Logger::Logger()
{
    // 默认构造函数
    m_target = terminal;
    m_level = debug;
}
Logger::Logger(log_target target, log_level level, const std::string &path)
{
    m_target = target;
    m_path = path;
    m_level = level;

    std::string strContent = "[" + currTime() + "]" + "<开始日志>\n";
    if (target != terminal)
    {
        m_outfile.open(path, std::ios::out | std::ios::app); // 打开输出文件
        m_outfile << strContent;
    }
    if (target != file)
    {
        // 如果日志对象不是仅文件
        std::cout << strContent;
    }
}
Logger::~Logger()
{
    std::string strContent = "[" + currTime() + "]" + "<结束日志>\n";
    if (m_outfile.is_open())
    {
        m_outfile << strContent;
    }
    m_outfile.flush();
    m_outfile.close();
}
void Logger::DEBUG(const std::string &text)
{
    output(text, debug);
}
void Logger::INFO(const std::string &text)
{
    output(text, info);
}
void Logger::WARNING(const std::string &text)
{
    output(text, warning);
}
void Logger::ERRORS(const std::string &text)
{
    output(text, error);
}
void Logger::output(const std::string &text, log_level act_level)
{
    std::string prefix;
    if (act_level == debug)
        prefix = "[DEBUG]";
    else if (act_level == info)
        prefix = "[INFO]";
    else if (act_level == warning)
        prefix = "[WARNING]";
    else if (act_level == error)
        prefix = "[ERROR]";
    // else prefix = "";
    // prefix += __FILE__;
    // prefix += " ";
    std::string outputContent = "[" + currTime() + "] " + prefix + ": " + text + "\n";
    if (m_level <= act_level && m_target != file)
    {
        // 当前等级设定的等级才会显示在终端，且不能是只文件模式
        std::cout << outputContent;
    }
    if (m_target != terminal)
        m_outfile << outputContent;

    m_outfile.flush(); // 刷新缓冲区
}
// [日志实现]<