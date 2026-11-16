SUMMARY = "sudo policy for cam_system images"
DESCRIPTION = "Sets the PATH that sudo gives the commands it runs. It holds no user rules, \
so it stays in release images; the debug user's rule belongs to ssh-access-dev."
SECTION = "PETALINUX/apps"
LICENSE = "MIT"
LIC_FILES_CHKSUM = "file://${COMMON_LICENSE_DIR}/MIT;md5=0835ade698e0bcf8506ecda2f7b4f302"

SRC_URI = "file://00-secure-path"

S = "${WORKDIR}"

RDEPENDS:${PN} += "sudo"

do_install() {
    install -d -m 0750 ${D}${sysconfdir}/sudoers.d
    install -m 0440 ${WORKDIR}/00-secure-path ${D}${sysconfdir}/sudoers.d/00-secure-path
}

FILES:${PN} += "${sysconfdir}/sudoers.d/00-secure-path"
