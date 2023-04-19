# ros_ndi_sensors
```python
pip3 install "git+https://github.com/SciKit-Surgery/scikit-surgerynditracker"
```
And then, modify `nditracker.py` file from
```python
if firmware in (' AURORA Rev 007', ' AURORA Rev 008',
                ' Polaris Vega 008', ' Polaris Spectra Rev 006'):
```
to
```python
if firmware in (' AURORA Rev 007', ' AURORA Rev 008',
                ' Polaris Vega 008', ' Polaris Spectra Rev 006', ' Polaris Spectra Rev 007'):
```