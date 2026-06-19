#
# Makefile
#

CIRCLEHOME = ../..

OBJS	= firmwarertc.o mcp7941x.o ds3231.o

librtc.a: $(OBJS)
	@echo "  AR    $@"
	@rm -f $@
	@$(AR) cr $@ $(OBJS)

include $(CIRCLEHOME)/Rules.mk

-include $(DEPS)
