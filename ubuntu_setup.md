## Useful Ubuntu Settings  
(Ubuntu 22.04 기준)  

### Alias  
`~/.bashrc` 파일은 터미널을 실행할 때마다 수행되는 파일로 이 파일을 수정하면 터미널에서 단축키와 같은 여러 설정이 가능하다. 다음 명령을 통해 `~/.bashrc` 파일을 편집할 수 있다.
```
gedit ~/.bashrc
```
텍스트 에디터에 나오는 명령어들은 새 터미널을 실행할 때 실행되는 명령이다. 마지막에 다음 명령어를 추가해 준 후 저장하자.
```
# aliases
alias eb='gedit ~/.bahsrc`
alias sb='source ~/.bashrc`
```
위와 같이 입력 후 저장한 후 터미널을 종료 후 재시작하거나 다음 명령을 입력하여 적용한다.
```
source ~/.bashrc
```
적용하면 `eb`만 입력하면 `gedit ~/.bashrc`를 입력한 것처럼 `~/.bashrc`파일을 편집할 수 있다.  
다른 단축어를 지정하려면 `alias [shortcut]='[command]'` 문법을 이용하여 `~/.bashrc`에 추가 후 `sb`를 입력하면 적용할 수 있다.

### Korean Setting
한국어 입력기 설정 방법이다. 설정에 들어가 다음 순서로 진입한다. `Setting > Region & Language > Manage Installed Languages > Install` Install이 완료되면 재부팅 후 `IBus` 환경설정에서 한글 입력기를 추가해준다.  
```
ibus-setup
```
추가 방법은 `IBus Preferences > Input Method > Add > Korean > Hangul > Add`  
한글을 추가하였다면 설정-키보드에서 한글 입력기를 적용해준다. `Setting > Keyboard > Input Sources > + > ... (Korean 검색) > Korean > Korean(Hangul) > Add`  
한글 적용 후 원래 있던 English는 `Remove`해 주도록 하자.  
기본적으로 한/영 변환은 한/영키(`Hangul`)와 `Shift+Space`로 설정되어 있으나 요즘 키보드는 한/영키가 우측 Alt 키로 매핑되어있기 때문에 다음 설정을 통해 변경해주도록 하자. `Korean (Hangul) > ... > Preferences > Hangul Toggle Key > Add > (한/영키로 사용할 버튼 입력) > OK` 한/영키 적용 후 사용하지 않을 `Shift+Space`는 삭제하는 것이 편하다.  
22.04 이전 버전은 [이곳](https://blu-y.github.io/carvis/guide/ubuntu_setting#41-korean-setting) 참조

### Ubuntu Kakao Mirror
우분투를 사용할 때 보통 `kr.archive.ubuntu.com` 아카이브 서버를 이용하여 데이터를 다운받게 되는데 속도가 조금 답답하다. 우분투 아카이브 서버의 내용을 그대로 반영하고 있는 카카오 미러 서버를 이용하면 조금 더 빠르게 우분투 아카이브의 자료를 업데이트하거나 다운로드 할 수 있다.  
방법은 `/etc/apt/sources.list`의 `kr.archive.ubuntu.com`을 `mirror.kakao.com`으로 바꾸면 된다. 기본 텍스트 에디터를 이용하여 손으로 바꿀 수 있지만 Vi 에디터를 이용하면 한번에 바꿀 수 있다.  
```
sudo vi /etc/apt/sources.list
```
Vi 에디터는 키보드로 컨트롤 가능한 여러 기능을 제공하기 때문에 익숙해지면 편하게 텍스트를 편집할 수 있기 때문에 자주 사용된다.  
`:`를 눌러야 입력모드에서 명령모드로 진입할 수 있으며 `%s` 명령을 통해 각 행의 처음 나오는 문구를 바꿀 수 있다. 다음 명령으로 `kr.archive.ubuntu.com`을 `mirror.kakao.com`으로 바꾸어준다.  
```
:%s/kr.archive.ubuntu.com/mirror.kakao.com
```
명령 모드에서 `w`는 저장, `q`는 종료, `!`는 강제를 의미한다. 다음 명령을 통해 저장하고 종료(강제)한다.  
```
:wq!
```
apt update를 시켜보면 이전보다 빨라진 것을 확인할 수 있다.  
```
sudo apt update
```

### Useful Softwares

[VS Code](https://code.visualstudio.com/)  
[Anaconda](https://www.anaconda.com/download#downloads)  