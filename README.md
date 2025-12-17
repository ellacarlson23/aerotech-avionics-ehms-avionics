# EHMS - Engine Health Monitoring System

[![DO-178C](https://img.shields.io/badge/DO--178C-Level%20B-blue)](https://en.wikipedia.org/wiki/DO-178C)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)](/)
[![Coverage](https://img.shields.io/badge/coverage-94%25-green)](/)
[![MISRA-C](https://img.shields.io/badge/MISRA--C-2012-orange)](/)

## Overview

The **Engine Health Monitoring System (EHMS)** is a certified avionics software solution providing real-time engine performance monitoring, predictive maintenance analytics, and health trend analysis for commercial and business aviation turbofan engines.

**Certification Level:** DO-178C Design Assurance Level B (Hazardous)

## Key Features

-  Real-time monitoring of 48+ engine parameters at 100Hz sample rate
-  Predictive maintenance using ML-based anomaly detection
-  Multi-level alert system (Advisory, Caution, Warning)
-  500+ flight hour data retention with AES-256 encryption
-  ARINC 429 and MIL-STD-1553B bus interfaces
-  ACARS/SATCOM ground station connectivity

## Supported Platforms

| OEM | Aircraft | Engine Types |
|-----|----------|--------------|
| Airbus | A320neo, A321neo, A321XLR | CFM LEAP-1A, PW1100G-JM |
| Embraer | E175-E2, E190-E2, E195-E2 | PW1700G, PW1900G |
| Bombardier | Global 7500, Global 8000 | GE Passport |
| Gulfstream | G700, G800 | Rolls-Royce Pearl 700 |

## Repository Structure

```
ehms-avionics/
├── src/
│   ├── core/                    # Core monitoring algorithms
│   │   ├── data_acquisition.c
│   │   ├── parameter_processing.c
│   │   ├── health_assessment.c
│   │   └── trend_analysis.c
│   ├── interfaces/              # Hardware interfaces
│   │   ├── arinc429/
│   │   ├── milstd1553/
│   │   └── discrete_io/
│   ├── alerts/                  # Alert management
│   │   ├── alert_manager.c
│   │   ├── alert_prioritization.c
│   │   └── crew_notification.c
│   ├── storage/                 # Data recording
│   │   ├── flight_recorder.c
│   │   ├── nvram_driver.c
│   │   └── encryption.c
│   ├── display/                 # Display formatting
│   │   ├── eicas_interface.c
│   │   ├── synoptic_pages.c
│   │   └── trend_graphics.c
│   ├── comms/                   # Communications
│   │   ├── acars_handler.c
│   │   ├── satcom_interface.c
│   │   └── ground_link.c
│   ├── predictive/              # Predictive maintenance
│   │   ├── anomaly_detection.c
│   │   ├── remaining_life.c
│   │   └── maintenance_scheduler.c
│   └── bit/                     # Built-In Test
│       ├── pbit.c
│       ├── cbit.c
│       └── ibit.c
├── include/                     # Header files
│   ├── ehms_types.h
│   ├── ehms_config.h
│   ├── arinc429_labels.h
│   └── engine_parameters.h
├── tests/                       # Test suites
│   ├── unit/
│   ├── integration/
│   └── hlr_verification/
├── docs/                        # Documentation
│   ├── SRS/                     # Software Requirements Spec
│   ├── SDD/                     # Software Design Document
│   ├── SVP/                     # Software Verification Plan
│   └── PSAC/                    # Plan for Software Aspects
├── tools/                       # Build & analysis tools
│   ├── static_analysis/
│   ├── code_coverage/
│   └── requirements_tracing/
├── config/                      # Configuration files
│   ├── engine_profiles/
│   └── oem_configurations/
├── Makefile
├── CMakeLists.txt
└── README.md
```

## Build Requirements

- **Compiler:** Green Hills MULTI or Wind River VxWorks 7
- **Target:** PowerPC MPC8548 or ARM Cortex-R5
- **RTOS:** VxWorks 7 or INTEGRITY-178 tuMP
- **Static Analysis:** Polyspace, LDRA TBvision
- **Coverage:** LDRA TBcover (MC/DC)

## Quick Start

```bash
# Clone repository
git clone https://github.com/aerotech-avionics/ehms-avionics.git

# Configure for target platform
./configure --target=ppc8548 --oem=airbus --aircraft=a320neo

# Build
make all

# Run static analysis
make analyze

# Execute unit tests
make test

# Generate coverage report
make coverage
```

## Compliance & Certification

### DO-178C Objectives

| Objective | Status |
|-----------|--------|
| Software Planning | ✅ Complete |
| Software Development | 🔄 In Progress |
| Software Verification | 🔄 In Progress |
| Configuration Management | ✅ Complete |
| Quality Assurance | ✅ Complete |
| Certification Liaison | 🔄 In Progress |

### Coding Standards

- MISRA-C:2012 (all mandatory rules, selected advisory)
- JSF++ AV (C++ subset where applicable)
- NASA/JPL Power of Ten Rules

## Testing

```bash
# Unit tests (host-based)
make test-unit

# Integration tests (target simulator)
make test-integration

# HLR verification tests
make test-hlr

# Full regression suite
make test-all
```

## Configuration

Engine profiles are stored in `config/engine_profiles/`:

```yaml
# config/engine_profiles/leap1a.yaml
engine:
  manufacturer: CFM
  model: LEAP-1A
  thrust_rating: 32000  # lbf
  
parameters:
  n1:
    label: 0o310
    range: [0, 110]
    unit: percent
    sample_rate: 100  # Hz
    
  egt:
    label: 0o311
    range: [0, 1200]
    unit: celsius
    sample_rate: 50
```

## Contributing

This repository follows aviation software development standards. All contributions must:

1. Include requirements traceability
2. Pass static analysis with zero violations
3. Achieve 100% MC/DC coverage for new code
4. Include peer review sign-off
5. Update affected documentation

## License

Proprietary - AeroTech Avionics Inc. © 2025

## Contact

- **Program Manager:** aviation-programs@aerotech.example.com
- **Engineering Lead:** ehms-engineering@aerotech.example.com
- **Certification:** das-certification@aerotech.example.com
