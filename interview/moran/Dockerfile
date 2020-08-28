FROM coenvl/googletest

WORKDIR /tmp/cpp/
COPY . .
RUN make release -j4
RUN make test -j4

CMD /tmp/cpp/bin/release
