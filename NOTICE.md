# License scope and third-party notice

Unless a file or component states otherwise, project-authored source code,
scripts, documentation, and hardware handoff material are released under the
MIT License in `LICENSE`. This includes the project-authored portions of the
versioned XSA handoffs.

The root MIT License does not relicense third-party or vendor material:

- vendored cJSON (`third_party/cjson/`) retains its upstream MIT terms;
- vendored libfacedetection (`third_party/libfacedetection/`, including the
  model data compiled into `facedetectcnn-data.cpp`) retains its upstream
  BSD-3-Clause terms in `third_party/libfacedetection/LICENSE`;
- the vendored UltraFace wrapper (`third_party/ultraface/`) retains its
  upstream MIT terms in `third_party/ultraface/LICENSE`;
- libfacedetection and UltraFace are compiled only with
  `-DCAM_SYSTEM_ENABLE_CPU_DETECTORS=ON`; the default build uses the PL detector;
- Linux kernel modules are marked GPL-2.0-only;
- ncnn, libjpeg-turbo, OpenSSL, 8812bu, YOLOv6Lite-S and other external
  components or model artifacts remain subject to their applicable upstream terms;
- the UltraFace RFB-320 and SCRFD model files used by the optional CPU
  candidates are not distributed in this repository;
- Xilinx-generated or Xilinx-owned content embedded in an XSA remains subject
  to the applicable Xilinx tool and IP terms;
- model provenance and known redistribution limitations are recorded in
  `docs/MODEL_ASSETS.md` and `docs/THIRD_PARTY_COMPONENTS.md`.

Nothing in the root MIT License grants rights that the project does not own.
