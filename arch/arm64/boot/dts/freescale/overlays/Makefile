# SPDX-License-Identifier: GPL-2.0

dtb-y += imx8mm-phyboard-polis-1532.1.dtbo
dtb-y += imx8mm-phyboard-polis-peb-av-010.dtbo
dtb-y += imx8mm-phyboard-polis-peb-av-010-tc358775.dtbo
dtb-y += imx8mm-phyboard-polis-peb-eval-01.dtbo
dtb-y += imx8mm-phycore-rpmsg.dtbo
dtb-y += imx8mm-phycore-no-eth.dtbo
dtb-y += imx8mm-phycore-no-spiflash.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rs232.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rs485.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rs485-switchable.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rts-cts.dtbo
dtb-y += imx8mn-phyboard-polis-peb-eval-01.dtbo
dtb-y += imx8mn-phyboard-polis-peb-av-010.dtbo
dtb-y += imx8mn-phyboard-polis-1532.1.dtbo
dtb-y += imx8mn-phycore-no-spiflash.dtbo
dtb-y += imx8mn-phycore-no-eth.dtbo
dtb-y += imx8mp-phyboard-pollux-peb-av-010.dtbo
dtb-y += imx8mp-phyboard-pollux-peb-av-012.dtbo
dtb-y += imx8mp-phyboard-pollux-peb-wlbt-05.dtbo
dtb-y += imx8mp-phyboard-pollux-1552.1.dtbo
dtb-y += imx8mp-phycore-no-eth.dtbo
dtb-y += imx8mp-phycore-no-rtc.dtbo
dtb-y += imx8mp-phycore-no-spiflash.dtbo
dtb-y += imx8mp-phycore-rpmsg.dtbo

DTC_FLAGS += -@

quiet_cmd_dtbo = DTBO     $@
cmd_dtbo = $(HOSTCC) -E $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ; \
	   $(DTC) -O dtb -o $@ -b 0 \
		$(addprefix -i,$(dir $<) $(DTC_INCLUDE)) $(DTC_FLAGS) \
		-d $(depfile).dtc.tmp $(dtc-tmp) ; \
	   cat $(depfile).pre.tmp $(depfile).dtc.tmp > $(depfile)

$(obj)/%.dtbo: $(src)/%.dtso $(DTC) FORCE
	$(call if_changed_dep,dtbo)
