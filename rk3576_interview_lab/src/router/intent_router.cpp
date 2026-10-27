#include "router/intent_router.h"



RouteDecision IntentRouter::route(const std::string& question)const
{

    if ((question.find("前面有什么")!=std::string::npos) ||   question.find("描述当前画面")!=std::string::npos)
    {
        return
        {
            RequestType::Vision,"matched vision phrase"
        };
    }
    return
    {
        RequestType::Text,"default text rule"
    };
}



const char* requestTypeName(RequestType type)
{

    if(type ==RequestType::Vision)
    return "VISION";

    else 
    return "TEXT";

}