rem see https://bitbucket.org/cskilbeck/vault/src/master/aws.txt
rem Access Key ID:          XXXX
rem Secret Access Key:      XXXX

rem `aws configure` to enter credentials
rem aws and node must be on path

node --check index.mjs
if errorlevel 1 goto done
del lambda.zip
"c:\Program Files\7-Zip\7z.exe" a -mmt8 -mx3 -tzip lambda.zip node_modules *.mjs
aws lambda update-function-code --function-name "water_softener" --zip-file "fileb://./lambda.zip" --publish 
:done
