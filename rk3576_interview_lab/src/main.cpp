
#include <iostream>
#include <string>
#include "router/intent_router.h"

int main()
{
    std::cout << "RK3576 Interview Lab\n";


    
    std::string question;
    IntentRouter router;
    


    while (true)
    {
        std::cout << "\nUser> " << std::flush;

        if (!std::getline(std::cin, question))
        {
            break;
        }

        if (question == "exit")
        {
            std::cout << "Goodbye!\n";
            break;
        }

        RouteDecision result=router.route(question);



        std::cout << "Echo: " << question << '\n';

        std::cout<<"Request Type:" << requestTypeName(result.type)<< '\n';

        std::cout<<"Reason:" << result.reason << '\n';
    }

    return 0;
}
