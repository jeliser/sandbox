
There is some combination of all these mechanisms that will allow me to have a captured lambda function sent in as a function pointer to the signal handler.

http://bannalia.blogspot.com/2016/07/passing-capturing-c-lambda-functions-as.html
https://en.cppreference.com/w/cpp/utility/functional/function/target
https://stackoverflow.com/questions/12587621/signal-handler-sa-sigaction-arguments


int32_t temp_val = 0;
inline int temp_func(int, int) {
  temp_val += 100;
  return temp_val;
}

inline void sig_handler(int, siginfo_t* info, void*)
{
    printf("Got here: %d\n", info->si_value.sival_int);
    temp_val = info->si_value.sival_int;
};

// Spawn a new process as a daemon -> double fork()/exec()
pid_t spawn(const std::string& cmd, const std::string& working_dir, ErrorCode& ec)
{
    // If we have an incoming working directory, check that it exists before forking
    if(!working_dir.empty() && !FileSystem::exists(working_dir))
    {
        ec = ENOENT;
        return -1;
    }

    // Fork the process
    ec = 0;
    auto pid  = getpid();
    auto cpid = fork(ec);

    // We are the parent process, so return the grandchild PID (or the error if failed)
    if(cpid != 0)
    {
        // There was an error, so return as failure
        if(cpid < 0)
        {
            return cpid;
        }

        auto l1 = [] (int, int) -> int { return 1; };
        auto f1 = std::function<int(int, int)>(std::bind(l1, std::placeholders::_1, std::placeholders::_2));
        std::function<int(int, int)> b1 = std::bind(l1, std::placeholders::_1, std::placeholders::_2);
        std::cout << l1(1, 2) << "  " << f1.target<int(*)(int, int)>() << "  " << b1.target<int(*)(int, int)>() << std::endl;
        auto f2 = std::function<int(int, int)>(temp_func);
        std::cout << f2(1, 2) << "  " << temp_val << "  " << f2.target<int(*)(int, int)>() << std::endl;

#if 1
        int num_callbacks=0;
        auto callback = [&num_callbacks] (int indent) -> void {
            num_callbacks += indent;
            std::cout<<"callback called "<<num_callbacks<<" times \n";
        };
        callback(100);
        auto thunk1 = [] (void* func, int indent) { // note thunk is captureless
            (*static_cast<decltype(callback)*>(func))(indent);
        };
        thunk1(&callback, 100);
#endif

        // Get the grandchild PID (eg: the daemon PID)
        int32_t gcpid = -1;
#if 1
#if 0
        //auto lambda   = [ &gcpid ] (int, siginfo_t* info, void*) -> void { gcpid = info->si_value.sival_int; };
        auto lambda   = [] (int, siginfo_t* info, void*) -> void { printf("Got here: %d\n", info->si_value.sival_int); };

        // We are making a captureless thunk ... new thing for me
        // http://bannalia.blogspot.com/2016/07/passing-capturing-c-lambda-functions-as.html
        auto thunk = [] (void* func, int arg1, siginfo_t* arg2, void* arg3) -> void
        {
            (*static_cast<decltype(lambda)*>(func))(arg1, arg2, arg3);
        };
        //auto thunk = [] (void (*func) (int, siginfo_t*, void*), int arg1, siginfo_t* arg2, void* arg3 ) -> void
        //{
        //    (*static_cast<decltype(lambda)*>(func))(arg1, arg2, arg3);
        //};
#endif
#if 1
        //auto lambda = [ &gcpid ] (int, siginfo_t* info, void*) -> void { gcpid = info->si_value.sival_int; printf("Got here: %d\n", gcpid); };
        //auto func   = std::bind(lambda, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);
        //auto func   = std::function<void (int, siginfo_t*, void*)>(std::bind(lambda, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        auto func   = std::function<void (int, siginfo_t*, void*)>(sig_handler);
#endif

        // Load the signal handler
        struct sigaction act;
        memset (&act, '\0', sizeof(act));
        //act.sa_sigaction = lambda;
        //act.sa_sigaction = &thunk;
        act.sa_sigaction = *(func.target<void (*) (int, siginfo_t* info, void*)>());
        std::cout << func.target<void (*) (int, siginfo_t* info, void*)>() << std::endl;
        act.sa_flags     = SA_SIGINFO;
        sigaction(SIGUSR2, &act, NULL);
#endif

        // Wait for the child process to terminate
        wait(NULL);
        signal(SIGUSR2, SIG_DFL);

        std::cout << temp_val << std::endl;

        return gcpid;
    }


