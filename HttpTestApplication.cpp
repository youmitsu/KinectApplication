// HttpTestApplication.cpp : �R���\�[�� �A�v���P�[�V�����̃G���g�� �|�C���g���`���܂��B
//

#include "windows.h"
#include "Wininet.h"

#include <tchar.h>
#include <assert.h>
#include <string>
#include <string.h>
#include <sstream>
#include <locale.h>
#include <conio.h>
#include "NhConversion.h"
#include <random>

#pragma comment(lib, "Wininet.lib")

typedef std::basic_string<TCHAR>		tstring;
typedef std::basic_stringstream<TCHAR>	tstringstream;

#define URLBUFFER_SIZE		(4096)
#define	READBUFFER_SIZE		(4096)
#define FEATURE_SIZE 5
#define FRAME_SIZE 13

bool HttpRequest(tstring strUserAgent,
	tstring strUrl,
	bool bIsHttpVerbGet,
	tstring strParameter,
	tstring& rstrResult)
{
	// �A�E�g�v�b�g�̏�����
	rstrResult = tstring();

	// �C���v�b�g�̃`�F�b�N
	if (0 == strUrl.length())
	{
		assert(!"URL���s��");
		return false;
	}

	// �ϐ�
	HINTERNET			hInternetOpen = NULL;
	HINTERNET			hInternetConnect = NULL;
	HINTERNET			hInternetRequest = NULL;
	char*				pszOptional = NULL;
	URL_COMPONENTS		urlcomponents;
	tstring				strServer;
	tstring				strObject;
	INTERNET_PORT		nPort;
	tstring				strVerb;
	tstring				strHeaders;
	tstringstream		ssRead;

	// URL���
	ZeroMemory(&urlcomponents, sizeof(URL_COMPONENTS));
	urlcomponents.dwStructSize = sizeof(URL_COMPONENTS);
	TCHAR szHostName[URLBUFFER_SIZE];
	TCHAR szUrlPath[URLBUFFER_SIZE];
	urlcomponents.lpszHostName = szHostName;
	urlcomponents.lpszUrlPath = szUrlPath;
	urlcomponents.dwHostNameLength = URLBUFFER_SIZE;
	urlcomponents.dwUrlPathLength = URLBUFFER_SIZE;
	if (!InternetCrackUrl(strUrl.c_str(),
		(DWORD)strUrl.length(),
		0,
		&urlcomponents))
	{	// URL�̉�͂Ɏ��s
		assert(!"URL��͂Ɏ��s");
		return false;
	}
	strServer = urlcomponents.lpszHostName;
	strObject = urlcomponents.lpszUrlPath;
	nPort = urlcomponents.nPort;

	// HTTP��HTTPS������ȊO��
	DWORD dwFlags = 0;
	if (INTERNET_SCHEME_HTTP == urlcomponents.nScheme)
	{	// HTTP
		dwFlags = INTERNET_FLAG_RELOAD				// �v�����ꂽ�t�@�C���A�I�u�W�F�N�g�A�܂��̓t�H���_�ꗗ���A�L���b�V������ł͂Ȃ��A���̃T�[�o�[���狭���I�Ƀ_�E�����[�h���܂��B
			| INTERNET_FLAG_DONT_CACHE			// �Ԃ��ꂽ�G���e�B�e�B���L���V���֒ǉ����܂���B
			| INTERNET_FLAG_NO_AUTO_REDIRECT;	// HTTP �����Ŏg�p����A���_�C���N�g�� HttpSendRequest �ŏ�������Ȃ����Ƃ��w�肵�܂��B
	}
	else if (INTERNET_SCHEME_HTTPS == urlcomponents.nScheme)
	{	// HTTPS
		dwFlags = INTERNET_FLAG_RELOAD				// �v�����ꂽ�t�@�C���A�I�u�W�F�N�g�A�܂��̓t�H���_�ꗗ���A�L���b�V������ł͂Ȃ��A���̃T�[�o�[���狭���I�Ƀ_�E�����[�h���܂��B
			| INTERNET_FLAG_DONT_CACHE			// �Ԃ��ꂽ�G���e�B�e�B���L���V���֒ǉ����܂���B
			| INTERNET_FLAG_NO_AUTO_REDIRECT	// HTTP �����Ŏg�p����A���_�C���N�g�� HttpSendRequest �ŏ�������Ȃ����Ƃ��w�肵�܂��B
			| INTERNET_FLAG_SECURE						// ���S�ȃg�����U�N�V�������g�p���܂��B����ɂ��ASSL/PCT ���g���悤�ɕϊ�����AHTTP �v�������ŗL���ł��B 
			| INTERNET_FLAG_IGNORE_CERT_DATE_INVALID	// INTERNET_FLAG_IGNORE_CERT_DATE_INVALID�AINTERNET_FLAG_IGNORE_CERT_CN_INVALID
			| INTERNET_FLAG_IGNORE_CERT_CN_INVALID;		// �́A�ؖ����Ɋւ���x���𖳎�����t���O
	}
	else
	{
		assert(!"HTTP�ł�HTTPS�ł��Ȃ�");
		return false;
	}

	// GET��POST��
	if (bIsHttpVerbGet)
	{	// GET
		strVerb = _T("GET");
		strHeaders = _T("");
		if (0 != strParameter.length())
		{	// �I�u�W�F�N�g�ƃp�����[�^���u?�v�ŘA��
			strObject += _T("?") + strParameter;
		}
	}
	else
	{	// POST
		strVerb = _T("POST");
		strHeaders = _T("Content-Type: application/x-www-form-urlencoded");
		if (0 != strParameter.length())
		{	// �p�����[�^���A���M����I�v�V�����f�[�^�ɕϊ�����
			pszOptional = NhT2M(strParameter.c_str());	// char������ɕϊ�
		}
	}

	// WinInet�̏�����
	hInternetOpen = InternetOpen(strUserAgent.c_str(),
		INTERNET_OPEN_TYPE_PRECONFIG,
		NULL, NULL, 0);
	if (NULL == hInternetOpen)
	{
		assert(!"WinInet�̏������Ɏ��s");
		goto LABEL_ERROR;
	}

	// HTTP�ڑ�
	hInternetConnect = InternetConnect(hInternetOpen,
		strServer.c_str(),
		nPort,
		NULL,
		NULL,
		INTERNET_SERVICE_HTTP,
		0,
		0);
	if (NULL == hInternetConnect)
	{
		assert(!"HTTP�ڑ��Ɏ��s");
		goto LABEL_ERROR;
	}

	// HTTP�ڑ����J��
	hInternetRequest = HttpOpenRequest(hInternetConnect,
		strVerb.c_str(),
		strObject.c_str(),
		NULL,
		NULL,
		NULL,
		dwFlags,
		NULL);
	if (NULL == hInternetRequest)
	{
		assert(!"HTTP�ڑ����J���Ɏ��s");
		goto LABEL_ERROR;
	}

	// HTTP�v�����M
	if (!HttpSendRequest(hInternetRequest,
		strHeaders.c_str(),
		(DWORD)strHeaders.length(),
		(LPVOID)((char*)pszOptional),
		pszOptional ? (DWORD)(strlen(pszOptional) * sizeof(char)) : 0))
	{
		assert(!"HTTP�v�����M�Ɏ��s");
		goto LABEL_ERROR;
	}

	// HTTP�v���ɑΉ�����X�e�[�^�X�R�[�h�̎擾
	DWORD dwStatusCode;
	DWORD dwLength = sizeof(DWORD);
	if (!HttpQueryInfo(hInternetRequest,
		HTTP_QUERY_STATUS_CODE | HTTP_QUERY_FLAG_NUMBER,
		&dwStatusCode,
		&dwLength,
		0))
	{
		assert(!"HTTP�v���ɑΉ�����X�e�[�^�X�R�[�h�̎擾�Ɏ��s");
		goto LABEL_ERROR;
	}
	if (HTTP_STATUS_OK != dwStatusCode)
	{
		assert(!"�X�e�[�^�X�R�[�h��OK�łȂ�");
		goto LABEL_ERROR;
	}

	// HTTP�t�@�C���ǂݍ���
	char szReadBuffer[READBUFFER_SIZE + 1];
	while (1)
	{
		DWORD dwRead = 0;
		if (!InternetReadFile(hInternetRequest, szReadBuffer, READBUFFER_SIZE, &dwRead))
		{
			assert(!"HTTP�t�@�C���ǂݍ��݂Ɏ��s");
			goto LABEL_ERROR;
		}
		if (0 == dwRead)
		{
			break;
		}
		szReadBuffer[dwRead] = '\0';	// �I�[�����u\0�v�̕t��
		size_t length = dwRead + 1;
		LPWSTR	pszWideChar = (LPWSTR)malloc(length * sizeof(WCHAR));
		MultiByteToWideChar(CP_UTF8,	// CODE PAGE: UTF-8
			0,
			szReadBuffer,
			-1,
			pszWideChar,
			(int)length);	// UTF-8�������ANSI������ɕϊ�
		TCHAR* pszTchar = NhW2T(pszWideChar);	// WideChar�������TCHAR������ɕϊ�
		ssRead << pszTchar;	// �X�g���[��������ɗ�������
		free(pszTchar);
		free(pszWideChar);
	}

	// �X�g���[����������A�o�͕�����ɕϊ�
	rstrResult = ssRead.str().c_str();

	if (pszOptional){ free(pszOptional); }
	InternetCloseHandle(hInternetRequest);
	InternetCloseHandle(hInternetConnect);
	InternetCloseHandle(hInternetOpen);
	return true;

LABEL_ERROR:
	if (pszOptional){ free(pszOptional); }
	InternetCloseHandle(hInternetRequest);
	InternetCloseHandle(hInternetConnect);
	InternetCloseHandle(hInternetOpen);
	return false;
}
/*
enum PARAM{
	PROC_ID = 1,
	DEVISE_ID = 2,
    FEATURE_ID = 3,
	FRAME = 4,
	VALUE = 5
};

enum FEATURE{
	HIP = 1,
	LEFTKNEE = 2, 
	LEFTHEEL = 3,
	RIGHTKNEE = 4,
	RIGHTHEEL = 5
};

TCHAR* setProcIdStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("proc_id=%d"), value);
	return str;
}

tstring setDeviseIdStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("devise_id=%d"), value);
	return str;
}

tstring setFeatureIdStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("feature_id=%d"), value);
	return str;
}

tstring setFrameStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("frame=%d"), value);
	return str;
}

tstring setValueStr(float value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("value=%f"), value);
	return str;
}*/
/*
int _tmain(int argc, _TCHAR* argv[])
{
	tstring strUserAgent = _T("HttpRequestTest");
	tstring strUrl = _T("https://kinect-walking-api.herokuapp.com/index");
	bool bIsHttpVerbGet = true;

	const int proc_id = 104;
	const int devise_id = 1;
	//const int feature_size = 5;
	const int features[FEATURE_SIZE] = { HIP, LEFTKNEE, LEFTHEEL, RIGHTKNEE, RIGHTHEEL };
	//const int nFrame = 13;
	float values[FEATURE_SIZE][FRAME_SIZE] = {};
	for (int i = 0; i < FEATURE_SIZE; i++){
		for (int j = 0; j < FRAME_SIZE; j++){
			values[i][j] = rand();
		}
	}
	tstring anp = _T("&");
	tstring procIdParams = setProcIdStr(proc_id);
	tstring deviseIdParams = setDeviseIdStr(devise_id);
	tstring strResult;
	for (int i = 0; i < FEATURE_SIZE; i++){
		for (int j = 0; j < FRAME_SIZE; j++){
			tstring featureIdParams = setFeatureIdStr(i+1);
			tstring frameParams = setFrameStr(j+1);
			tstring valueParams = setValueStr(values[i][j]);
			tstring strParameter = procIdParams + anp + deviseIdParams + anp + featureIdParams + anp + frameParams + anp + valueParams;
			if (!HttpRequest(strUserAgent, strUrl, bIsHttpVerbGet,strParameter, strResult))
			{
				return false;
			}
		}
	}

	setlocale(LC_ALL, "Japanese");
	_tprintf(_T("%s"), strResult.c_str());

	_getch();

	return 0;
}*/