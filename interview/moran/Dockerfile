FROM coenvl/googletest

WORKDIR /tmp/cpp/
COPY . .
RUN make moran -j4
RUN make test -j4

CMD /tmp/cpp/bin/moran
