
### Initial configuration

install the following packages
`sudo apt-get install build-essential devscripts ubuntu-dev-tools debhelper dh-make patch cdbs quilt gnupg fakeroot lintian`

##### DPUT config
make a ~/.dput.cf file and input the following:
```
[ur-rtde]
fqdn = ppa.launchpad.net
method = ftp
incoming = ~sdurobotics/ur-rtde/ubuntu/
login = <launchpad user id>
allow_unsigned_uploads = 0
```
replace `<launchpad user id>`with your username

##### gbp config 

Install git build package
`sudo apt-get install git-buildpackage`

add the following lines to .bashrc and change the name and email address to your own

```
export DEBFULLNAME="Kasper Høj Lorenzen" 
export DEBEMAIL="kalor@mmmi.sdu.dk"
```
##### gpg key

to upload a package to launchpad you need a gpg key. Use the following to tutorial to set up a gpg key or follow along 
for a brief overview.
https://help.ubuntu.com/community/GnuPrivacyGuardHowto#Using_GnuPG_to_generate_a_key

```
gpg --gen-key
# Options:
# "Please select what kind of key you want:" select (1)
# "What keysize do you want? " selecting (2048)  should be fine
# "Key is valid for?" selecting (0) means it is valid forever and you need to revoke it
# Verify your email and name with (Y) 
# You need a Passphrase to protect your secret key. type 0 to continue then enter you passphrase twice

# when done you should see somthing like:

gpg: key D8FC66D2 marked as ultimately trusted
public and secret key created and signed.

pub   1024D/D8FC66D2 2005-09-08
      Key fingerprint = 95BD 8377 2644 DD4F 28B5  2C37 0F6E 4CA6 D8FC 66D2
uid                  Dennis Kaarsemaker (Tutorial key) <dennis@kaarsemaker.net>
sub   2048g/389AA63E 2005-09-08
```

add the following line to your .bashrc where the key is the key fingerprint from the above text
```
export UR_RTDE_DEB_KEY=95BD83772644DD4F28B52C370F6E4CA6D8FC66D2
```

if you continue to work in the same terminal remember to:
```
source ~/.bashrc
```

next step is to upload the key to the key server
```
gpg --send-keys --keyserver keyserver.ubuntu.com $UR_RTDE_DEB_KEY
```

### Uploading a package to launchpad

go to the root of the git repository and run:
`./debian/ur_rtde_build.sh -c deb-upload`
This script will take you through the whole procedure of uploading the package
the -c option is for configuring the changelog before doing a upload.

To test if the .deb files will build run 
`./debian/ur_rtde_build.sh deb`

