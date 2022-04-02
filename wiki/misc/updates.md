Wiki | [Home](../../README.md) | [Documentation](../documentation/documentation.md) | [Tutorials](../tutorials/tutorials.md) | [Qualifiers](../qualifiers/qualifier.md) | [Finals](../finals/finals.md)

---

## Release 2022, April 1

- Updated the [Scoring](../documentation/scoring.md) section to include more information on how movable trays affect the scoring.
- Only one table of movable trays is now present. The other table was removed because it was obstructing assembly at assembly station 1 (as1). The way to specify movable trays in yaml files has also been modified. The new way is shown below and is described in the [YAML Configuration Files](../documentation/configuration_files.md) section.
  
```yaml
models_over_tray_table:
  tray_table:
    tray_1: movable_tray_metal_shiny
    tray_2: movable_tray_metal_rusty
    tray_3: movable_tray_dark_wood
```

## Release 2022, March 26


* Wiki released along with the software.
* Please report any broken links on the wiki at zeid.kootbally@gmail.com
