SUMMARY = "Camera system with camera streaming, tracking and detector candidates"
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://cam_system-src.tar.gz \
           file://cam-system-artifacts.sha256 \
           file://verify_elf.sh \
           "

S = "${WORKDIR}/cam_system-src"

inherit cmake pkgconfig

# The product build uses the PL detector only; ncnn is needed only with
# -DCAM_SYSTEM_ENABLE_CPU_DETECTORS=ON.
DEPENDS = "libjpeg-turbo openssl"

EXTRA_OECMAKE += " \
    -DCAM_SYSTEM_DEPENDENCY_MODE=YOCTO \
    -DCAM_SYSTEM_ENABLE_WARNINGS=ON \
    -DCAM_SYSTEM_ENABLE_CPU_DETECTORS=OFF \
    -DCMAKE_SKIP_RPATH=ON \
    "

RDEPENDS:${PN} += "libturbojpeg libssl libcrypto libstdc++ libgcc cam-face-accel"

do_compile:append() {
    ${WORKDIR}/verify_elf.sh ${B}/cam_system
}

CAM_SYSTEM_MODEL_DIR = "${datadir}/cam-system/models"
CAM_SYSTEM_MANIFEST_DIR = "${datadir}/cam-system/manifest"

do_install:append() {
    install -d ${D}${CAM_SYSTEM_MANIFEST_DIR}
    install -m 0644 ${WORKDIR}/cam-system-artifacts.sha256 ${D}${CAM_SYSTEM_MANIFEST_DIR}/cam-system-artifacts.sha256
}

FILES:${PN} += " \
    ${bindir}/cam_system \
    ${CAM_SYSTEM_MODEL_DIR} \
    ${CAM_SYSTEM_MANIFEST_DIR}/cam-system-artifacts.sha256 \
    ${sysconfdir}/cam_system.conf \
    "

CONFFILES:${PN} += "${sysconfdir}/cam_system.conf"
