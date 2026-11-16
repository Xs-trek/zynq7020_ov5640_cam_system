SUMMARY = "Zynq-7020 face accelerator driver and verified weights"
SECTION = "kernel/modules"
LICENSE = "GPL-2.0-only"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/GPL-2.0-only;md5=801f80980d171dd6425610833a22dbe6"

SRC_URI = "file://Makefile \
           file://cam-face-accel.c \
           file://cam_face_accel_uapi.h \
           file://face_accel_regs.h \
           file://unified128_dw32_weights_v1.bin \
           "

SRC_URI[unified128_dw32_weights_v1.bin.sha256sum] = "93ae933897bb510a2a47a34a7ee27249a66904cc09964a1938187891239cbfab"

S = "${WORKDIR}"
B = "${S}"

inherit module

KERNEL_MODULE_AUTOLOAD += "cam-face-accel"

do_install() {
    install -d ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra
    install -m 0644 ${B}/cam-face-accel.ko \
        ${D}${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/cam-face-accel.ko

    install -d ${D}${nonarch_base_libdir}/firmware/cam-system/face-accel
    install -m 0644 ${WORKDIR}/unified128_dw32_weights_v1.bin \
        ${D}${nonarch_base_libdir}/firmware/cam-system/face-accel/unified128_dw32_weights_v1.bin

    install -d ${D}${includedir}/linux
    install -m 0644 ${WORKDIR}/cam_face_accel_uapi.h \
        ${D}${includedir}/linux/cam_face_accel_uapi.h
}

FILES:${PN} += " \
    ${nonarch_base_libdir}/modules/${KERNEL_VERSION}/extra/cam-face-accel.ko \
    ${nonarch_base_libdir}/firmware/cam-system/face-accel/unified128_dw32_weights_v1.bin \
    "
FILES:${PN}-dev += "${includedir}/linux/cam_face_accel_uapi.h"

RPROVIDES:${PN} += "kernel-module-cam-face-accel"
