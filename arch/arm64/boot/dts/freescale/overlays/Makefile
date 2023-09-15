# SPDX-License-Identifier: GPL-2.0

dtb-y += imx8mm-phyboard-polis-1532.1.dtbo
dtb-y += imx8mm-phyboard-polis-peb-av-010-sn65dsi83.dtbo
dtb-y += imx8mm-phyboard-polis-peb-av-010-tc358775.dtbo
dtb-y += imx8mm-phyboard-polis-peb-av-013-sn65dsi83.dtbo
dtb-y += imx8mm-phyboard-polis-peb-av-013-tc358775.dtbo
dtb-y += imx8mm-phyboard-polis-peb-eval-01.dtbo
dtb-y += imx8mm-phycore-adin1300.dtbo
dtb-y += imx8mm-phycore-rpmsg.dtbo
dtb-y += imx8mm-phycore-no-eth.dtbo
dtb-y += imx8mm-phycore-no-spiflash.dtbo
dtb-y += imx8mm-phygate-tauri-analog-io.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rs232.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rs485.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rs485-switchable.dtbo
dtb-y += imx8mm-phygate-tauri-rs232-rts-cts.dtbo
dtb-y += imx8mm-vm016.dtbo
dtb-y += imx8mm-vm016-fpdlink-port0.dtbo
dtb-y += imx8mm-vm016-fpdlink-port1.dtbo
dtb-y += imx8mm-vm017.dtbo
dtb-y += imx8mm-vm017-fpdlink-port0.dtbo
dtb-y += imx8mm-vm017-fpdlink-port1.dtbo
dtb-y += imx8mm-vm020.dtbo
dtb-y += imx8mm-vm020-fpdlink-port0.dtbo
dtb-y += imx8mm-vm020-fpdlink-port1.dtbo
dtb-y += imx8mn-phyboard-polis-peb-eval-01.dtbo
dtb-y += imx8mn-phycore-no-spiflash.dtbo
dtb-y += imx8mn-phycore-no-eth.dtbo
dtb-y += imx8mp-isi-csi1.dtbo
dtb-y += imx8mp-isi-csi2.dtbo
dtb-y += imx8mp-isp-csi1.dtbo
dtb-y += imx8mp-isp-csi2.dtbo
dtb-y += imx8mp-phyboard-pollux-peb-av-010.dtbo
dtb-y += imx8mp-phyboard-pollux-peb-av-012.dtbo
dtb-y += imx8mp-phyboard-pollux-peb-wlbt-05.dtbo
dtb-y += imx8mp-phycore-adin1300.dtbo
dtb-y += imx8mp-phycore-no-eth.dtbo
dtb-y += imx8mp-phycore-no-rtc.dtbo
dtb-y += imx8mp-phycore-no-spiflash.dtbo
dtb-y += imx8mp-phycore-rpmsg.dtbo
dtb-y += imx8mp-vm016-csi1.dtbo
dtb-y += imx8mp-vm016-csi2.dtbo
dtb-y += imx8mp-vm016-csi1-fpdlink-port0.dtbo
dtb-y += imx8mp-vm016-csi1-fpdlink-port1.dtbo
dtb-y += imx8mp-vm016-csi2-fpdlink-port0.dtbo
dtb-y += imx8mp-vm016-csi2-fpdlink-port1.dtbo
dtb-y += imx8mp-vm017-csi1.dtbo
dtb-y += imx8mp-vm017-csi2.dtbo
dtb-y += imx8mp-vm017-csi1-fpdlink-port0.dtbo
dtb-y += imx8mp-vm017-csi1-fpdlink-port1.dtbo
dtb-y += imx8mp-vm017-csi2-fpdlink-port0.dtbo
dtb-y += imx8mp-vm017-csi2-fpdlink-port1.dtbo
dtb-y += imx8mp-vm020-csi1.dtbo
dtb-y += imx8mp-vm020-csi2.dtbo
dtb-y += imx8qxp-phytec-pcm-942-bt.dtbo
dtb-y += imx8qxp-phytec-pcm-942-lcd-018.dtbo
dtb-y += imx8qxp-phytec-pcm-942-lvds0-peb-lvds-01.dtbo
dtb-y += imx8qxp-phytec-pcm-942-lvds1-peb-lvds-01.dtbo
dtb-y += imx8qxp-phytec-pcm-942-parallel-camera.dtbo
dtb-y += imx8qxp-phytec-pcm-942-uart2.dtbo
dtb-y += imx8qxp-phytec-pcm-942-wifi.dtbo

DTC_FLAGS += -@

quiet_cmd_dtbo = DTBO     $@
cmd_dtbo = $(HOSTCC) -E $(dtc_cpp_flags) -x assembler-with-cpp -o $(dtc-tmp) $< ; \
	   $(DTC) -O dtb -o $@ -b 0 \
		$(addprefix -i,$(dir $<) $(DTC_INCLUDE)) $(DTC_FLAGS) \
		-d $(depfile).dtc.tmp $(dtc-tmp) ; \
	   cat $(depfile).pre.tmp $(depfile).dtc.tmp > $(depfile)

$(obj)/%.dtbo: $(src)/%.dtso $(DTC) FORCE
	$(call if_changed_dep,dtbo)
