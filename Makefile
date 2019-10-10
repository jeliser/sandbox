TYPE ?= RELEASE

all:: tests release

tests:: TYPE:=DEBUG
tests::
	@echo ${TYPE}

release::
	@echo ${TYPE}

