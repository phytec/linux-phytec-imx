# SPDX-License-Identifier: GPL-2.0

dtb-y += imx7d-phyboard-zeta-peb-av-02.dtbo
dtb-y += imx7d-phyboard-zeta-peb-av-02-rev0.dtbo
dtb-y += imx7d-phyboard-zeta-av-06.dtbo
dtb-y += imx7d-phyboard-zeta-peb-d-rpi.dtbo

dtb-y += imx7d-phyboard-zeta-m4.dtso

DTC_FLAGS += -@

quiet_cmd_dtbo = DTBO     $@
cmd_dtbo = $(HOSTCC) -E $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ; \
	   $(DTC) -O dtb -o $@ -b 0 \
		$(addprefix -i,$(dir $<) $(DTC_INCLUDE)) $(DTC_FLAGS) \
		-d $(depfile).dtc.tmp $(dtc-tmp) ; \
	   cat $(depfile).pre.tmp $(depfile).dtc.tmp > $(depfile)

$(obj)/%.dtbo: $(src)/%.dtso $(DTC) FORCE
	$(call if_changed_dep,dtbo)
