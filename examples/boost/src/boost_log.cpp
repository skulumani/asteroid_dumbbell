#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/expressions.hpp>

void init() {
    boost::log::core::get()->set_filter
    (
        boost::log::trivial::severity >= boost::log::trivial::info
    );
}

void init_log_to_file_basic() {
    boost::log::add_file_log("sample%N.log");
    
    boost::log::core::get()->set_filter(
            boost::log::trivial::severity >= boost::log::trivial::info);
    
    boost::log::add_common_attributes();
}

void init_log_to_file_advanced() {
    boost::log::add_file_log(
            boost::log::keywords::file_name = "sample%N.log", /**< file name patter */
            boost::log::keywords::rotation_size = 10 * 1024 * 1024, /**< rotate file every 10 MiB */
            boost::log::keywords::time_based_rotation = boost::log::sinks::file::rotation_at_time_point(0, 0, 0), /**<  rotate at midnight */
            boost::log::keywords::format = "[%TimeStamp%]: %Message%"); /**< log record format */

    boost::log::core::get()->set_filter(
            boost::log::trivial::severity >= boost::log::trivial::info);
}
int main(int, char*[]) {

    init_log_to_file_basic();
    
    boost::log::sources::severity_logger<boost::log::trivial::severity_level> lg;

    BOOST_LOG_SEV(lg, boost::log::trivial::trace) << "A trace severity message";
    BOOST_LOG_SEV(lg, boost::log::trivial::debug) << "A debug severity message";
    BOOST_LOG_SEV(lg, boost::log::trivial::info) << "An informational severity message";
    BOOST_LOG_SEV(lg, boost::log::trivial::warning) << "A warning severity message";
    BOOST_LOG_SEV(lg, boost::log::trivial::error) << "An error severity message";
    BOOST_LOG_SEV(lg, boost::log::trivial::fatal) << "A fatal severity message";

    return 0;
}
