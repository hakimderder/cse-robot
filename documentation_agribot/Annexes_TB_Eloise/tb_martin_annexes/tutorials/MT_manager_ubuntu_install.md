# Step by step installation of MT Manager for Xsens for ubuntu

1. The link below will take you to the dowload page for MT suites :
- [MT suites](https://www.xsens.com/cs/c/?cta_guid=d6a8454e-6db4-41e7-9f81-f8fc1c4891b3&signature=AAH58kEGRt-HjO-_B4RpFVk6-ZLOuf5bEQ&pageId=27796161161&placement_guid=e7ef7e11-db88-4d9e-b36e-3f937ea4ae15&click=9edf9fc8-722e-42d8-a0ff-d59bdf257560&hsutk=fef1a0dbcad427c681f2efdf4f33c898&canon=https%3A%2F%2Fwww.xsens.com%2Fsoftware-downloads&utm_referrer=https%3A%2F%2Fwww.google.com%2F&portal_id=3446270&redirect_url=APefjpEGB1TVqzydIzitzG1PPZsBr6jROufKhCeIaibXXK6HscV41ZMW2Tdzjbau2D0ynRMErzJFAOfSopaqvKoLdpjSEPOFiS-sN-hB958IECiusBcCPUPeYDS3fQNkpk8wdhBKA4q3fu8rD_lvo3Dbv3UD4tf-fwNDNqSwlRk9fTbdzu0QK2Wyg8LBFGSfPlO3moL0m6kGzJt4psRXBYaDbPQMNkIfA1vXlpbrXeN_oU0rUGp7sUrCDpeuOx5aipUOXApFZLdm363ZE7I_DvnmZ-ptUT0zDUM6v9_eUaAB4qydYTPoOMg&__hstc=81749512.fef1a0dbcad427c681f2efdf4f33c898.1627636119175.1628242182879.1628245774599.5&__hssc=81749512.4.1628245774599&__hsfp=1746458010&contentType=standard-page)

2. Extract the content at the approptiate location on your computer.

3. Make sure you already have the following packages installed :
   - QT5
   - libqt5opengl5
   - libusb-1.0-0
   - libxcb-xinerama0
   - libxcb-xinput0
   - libdouble-conversion1

4. Run those two scripts :
   - ./MT_Software_Suite_linux-x64_2021.0/mtsdk_linux-x64_2021.0.sh
   - ./MT_Software_Suite_linux-x64_2021.0/mfmsdk_linux-x64_2021.0.sh

5. Make sure the current user is in the dialout group, if not run : 
   - sudo usermod -G dialout -a $USER

6. Run the mtmanager in bin :
   - ./MT_Software_Suite_linux-x64_2021.0/mtmanager_linux-x64_2021.0/mtmanager/linux-x64/bin/mtmanager

7. If there is still issues please refer either to the MTM.README or the website

8. Eventually try :
   - sudo modprobe ftdi_sio
   - echo 2639 0301 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id


