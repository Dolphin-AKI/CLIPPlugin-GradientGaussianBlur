//! TriglavPlugIn SDK
//! Copyright (c) CELSYS Inc.
//! All Rights Reserved.
#include "TriglavPlugInSDK/TriglavPlugInSDK.h"
#include "PlugInCommon/PIHSVFilter.h"



/* ---------------------------------------------------------------------------
MIT License

Copyright (c) 2025/2/14  Akihiro.Watanabe, (Mitobe Hikane)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
--------------------------------------------------------------------------- */

#include <vector>
#include <cmath>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <ctime>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


#define ENABLE_LOGGING 0 // 1: Enable logging, 0: Disable logging


typedef	unsigned char	BYTE;

static	const int kItemKeyRadius					= 1;

static	const int kStringIDFilterCategoryName		= 101;
static	const int kStringIDFilterName				= 102;
static	const int kStringIDItemCaptionRadius		= 103;

struct	GaussianBlurFilterInfo
{
	TriglavPlugInDouble		radius;

	TriglavPlugInPropertyService*	pPropertyService;
};


enum LogLevel {
	INFO,
	WARNING,
	ERROR
};

void WriteLog(const std::string& message, LogLevel level) {
#if ENABLE_LOGGING
	std::ofstream logFile("C:\\logs\\plugin_log.txt", std::ios_base::app);
	if (logFile.is_open()) {
	// タイムスタンプを取得
	auto now = std::chrono::system_clock::now();
	std::time_t now_time = std::chrono::system_clock::to_time_t(now);
	char timeStr[26];
	ctime_s(timeStr, sizeof(timeStr), &now_time);
	timeStr[24] = '\0'; // 改行を削除

	// ログレベルを文字列に変換
	std::string levelStr;
	switch (level) {
	case INFO: levelStr = "INFO"; break;
	case WARNING: levelStr = "WARNING"; break;
	case ERROR: levelStr = "ERROR"; break;
	}

	// ログを書き出す
	logFile << "[" << timeStr << "] [" << levelStr << "] " << message << std::endl;
	logFile.close();
}
 else {
	 std::cerr << "Unable to open log file" << std::endl;
	}
#endif
}



//	プロパティコールバック
static void TRIGLAV_PLUGIN_CALLBACK TriglavPlugInFilterPropertyCallBack( TriglavPlugInInt* result, TriglavPlugInPropertyObject propertyObject, const TriglavPlugInInt itemKey, const TriglavPlugInInt notify, TriglavPlugInPtr data )
{
	(*result)	= kTriglavPlugInPropertyCallBackResultNoModify;

	GaussianBlurFilterInfo*	pFilterInfo	= static_cast<GaussianBlurFilterInfo*>(data);
	if( pFilterInfo != NULL )
	{
		TriglavPlugInPropertyService*	pPropertyService	= (*pFilterInfo).pPropertyService;
		if( pPropertyService != NULL )
		{
			if( notify == kTriglavPlugInPropertyCallBackNotifyValueChanged )
			{
				TriglavPlugInDouble	value;
				(*(*pFilterInfo).pPropertyService).getDecimalValueProc(&value,propertyObject,itemKey);

				if( itemKey == kItemKeyRadius )
				{
					if( (*pFilterInfo).radius != value )
					{
						(*pFilterInfo).radius	= value;
						(*result)	= kTriglavPlugInPropertyCallBackResultModify;
					}
				}
			}
		}
	}
}


// ガウスカーネルの計算
double Gaussian(double x, double y, double sigma) {
	return (1.0 / (2.0 * M_PI * sigma * sigma)) * std::exp(-(x * x + y * y) / (2.0 * sigma * sigma));
}

void TRIGLAV_PLUGIN_API TriglavPluginCall( TriglavPlugInInt* result, TriglavPlugInPtr* data, TriglavPlugInInt selector, TriglavPlugInServer* pluginServer, TriglavPlugInPtr reserved )
{
	*result	= kTriglavPlugInCallResultFailed;
	if( pluginServer != NULL )
	{
		if( selector == kTriglavPlugInSelectorModuleInitialize )
		{
			//	プラグインの初期化
			TriglavPlugInModuleInitializeRecord*	pModuleInitializeRecord	= (*pluginServer).recordSuite.moduleInitializeRecord;
			TriglavPlugInStringService*				pStringService			= (*pluginServer).serviceSuite.stringService;
			if( pModuleInitializeRecord != NULL && pStringService != NULL )
			{
				TriglavPlugInInt	hostVersion;
				(*pModuleInitializeRecord).getHostVersionProc(&hostVersion,(*pluginServer).hostObject);
				if( hostVersion >= kTriglavPlugInNeedHostVersion )
				{
					TriglavPlugInStringObject	moduleID	= NULL;
					const char*					moduleIDString	= "A9EE0802-84E7-4847-87D8-9EBAC916EEE4";
					(*pStringService).createWithAsciiStringProc(&moduleID,moduleIDString,static_cast<TriglavPlugInInt>(::strlen(moduleIDString)));
					(*pModuleInitializeRecord).setModuleIDProc((*pluginServer).hostObject,moduleID);
					(*pModuleInitializeRecord).setModuleKindProc((*pluginServer).hostObject,kTriglavPlugInModuleSwitchKindFilter);
					(*pStringService).releaseProc(moduleID);

					GaussianBlurFilterInfo*	pFilterInfo	= new GaussianBlurFilterInfo;
					*data	= pFilterInfo;
					*result	= kTriglavPlugInCallResultSuccess;
				}
			}
		}
		else if( selector == kTriglavPlugInSelectorModuleTerminate )
		{
			//	プラグインの終了処理
			GaussianBlurFilterInfo*	pFilterInfo	= static_cast<GaussianBlurFilterInfo*>(*data);
			delete pFilterInfo;
			*data	= NULL;
			*result	= kTriglavPlugInCallResultSuccess;
		}
		else if( selector == kTriglavPlugInSelectorFilterInitialize )
		{
			//	フィルタの初期化
			TriglavPlugInRecordSuite*				pRecordSuite			= &(*pluginServer).recordSuite;
			TriglavPlugInHostObject					hostObject				= (*pluginServer).hostObject;
			TriglavPlugInStringService*				pStringService			= (*pluginServer).serviceSuite.stringService;
			TriglavPlugInPropertyService*			pPropertyService		= (*pluginServer).serviceSuite.propertyService;
			if( TriglavPlugInGetFilterInitializeRecord(pRecordSuite) != NULL && pStringService != NULL && pPropertyService != NULL )
			{
				//	フィルタカテゴリ名とフィルタ名の設定
				TriglavPlugInStringObject	filterCategoryName	= NULL;
				TriglavPlugInStringObject	filterName			= NULL;
				(*pStringService).createWithStringIDProc(&filterCategoryName,kStringIDFilterCategoryName,(*pluginServer).hostObject);
				(*pStringService).createWithStringIDProc(&filterName,kStringIDFilterName,(*pluginServer).hostObject);
				
				TriglavPlugInFilterInitializeSetFilterCategoryName(pRecordSuite,hostObject,filterCategoryName,'c');
				TriglavPlugInFilterInitializeSetFilterName(pRecordSuite,hostObject,filterName,'h');
				(*pStringService).releaseProc(filterCategoryName);
				(*pStringService).releaseProc(filterName);

				//	プレビュー
				TriglavPlugInFilterInitializeSetCanPreview(pRecordSuite,hostObject,true);

				//	ターゲット
				TriglavPlugInInt	target[]={kTriglavPlugInFilterTargetKindRasterLayerRGBAlpha};
				TriglavPlugInFilterInitializeSetTargetKinds(pRecordSuite,hostObject,target,1);

				//	プロパティの作成
				TriglavPlugInPropertyObject	propertyObject;
				(*pPropertyService).createProc(&propertyObject);

				//	ぼかし範囲 (Radius)
				TriglavPlugInStringObject	radiusCaption	= NULL;
				(*pStringService).createWithStringIDProc(&radiusCaption,kStringIDItemCaptionRadius,(*pluginServer).hostObject);
				(*pPropertyService).addItemProc(propertyObject,kItemKeyRadius,kTriglavPlugInPropertyValueTypeDecimal,kTriglavPlugInPropertyValueKindDefault,kTriglavPlugInPropertyInputKindDefault,radiusCaption,'r');
				(*pPropertyService).setDecimalValueProc(propertyObject,kItemKeyRadius,1.0);
				(*pPropertyService).setDecimalDefaultValueProc(propertyObject,kItemKeyRadius,1.0);
				(*pPropertyService).setDecimalMinValueProc(propertyObject,kItemKeyRadius,0.01);
				(*pPropertyService).setDecimalMaxValueProc(propertyObject,kItemKeyRadius,10.0);
				(*pStringService).releaseProc(radiusCaption);

				//	プロパティの設定
				TriglavPlugInFilterInitializeSetProperty(pRecordSuite,hostObject,propertyObject);
				TriglavPlugInFilterInitializeSetPropertyCallBack(pRecordSuite,hostObject,TriglavPlugInFilterPropertyCallBack,*data);

				//	プロパティの破棄
				(*pPropertyService).releaseProc(propertyObject);

				*result	= kTriglavPlugInCallResultSuccess;
			}
		}
		else if( selector == kTriglavPlugInSelectorFilterTerminate )
		{
			//	フィルタの終了処理
			*result	= kTriglavPlugInCallResultSuccess;
		}
		else if( selector == kTriglavPlugInSelectorFilterRun )
		{
			//	フィルタの実行
			TriglavPlugInRecordSuite*		pRecordSuite		= &(*pluginServer).recordSuite;
			TriglavPlugInOffscreenService*	pOffscreenService	= (*pluginServer).serviceSuite.offscreenService;
			TriglavPlugInPropertyService*	pPropertyService	= (*pluginServer).serviceSuite.propertyService;
			TriglavPlugInBitmapService*		pBitmapService		= (*pluginServer).serviceSuite.bitmapService;
			if( TriglavPlugInGetFilterRunRecord(pRecordSuite) != NULL && pOffscreenService != NULL && pPropertyService != NULL && pBitmapService != NULL )
			{
				TriglavPlugInPropertyObject		propertyObject;
				TriglavPlugInFilterRunGetProperty(pRecordSuite,&propertyObject,(*pluginServer).hostObject);

				TriglavPlugInOffscreenObject	sourceOffscreenObject;
				TriglavPlugInFilterRunGetSourceOffscreen(pRecordSuite,&sourceOffscreenObject,(*pluginServer).hostObject);

				TriglavPlugInOffscreenObject	destinationOffscreenObject;
				TriglavPlugInFilterRunGetDestinationOffscreen(pRecordSuite,&destinationOffscreenObject,(*pluginServer).hostObject);

				TriglavPlugInRect				selectAreaRect;
				TriglavPlugInFilterRunGetSelectAreaRect(pRecordSuite,&selectAreaRect,(*pluginServer).hostObject);

				TriglavPlugInOffscreenObject	selectAreaOffscreenObject;
				TriglavPlugInFilterRunGetSelectAreaOffscreen(pRecordSuite,&selectAreaOffscreenObject,(*pluginServer).hostObject);

				TriglavPlugInInt	r,g,b;
				(*pOffscreenService).getRGBChannelIndexProc(&r,&g,&b,destinationOffscreenObject);
				
				TriglavPlugInInt	blockRectCount;				
				(*pOffscreenService).getBlockRectCountProc(&blockRectCount,destinationOffscreenObject,&selectAreaRect);
				
				std::vector<TriglavPlugInRect>	blockRects;
				blockRects.resize(blockRectCount);
				for( TriglavPlugInInt i=0; i<blockRectCount; ++i )
				{
					(*pOffscreenService).getBlockRectProc(&blockRects[i],i,destinationOffscreenObject,&selectAreaRect);
				}

				TriglavPlugInFilterRunSetProgressTotal(pRecordSuite,(*pluginServer).hostObject,blockRectCount);

				GaussianBlurFilterInfo*	pFilterInfo			= static_cast<GaussianBlurFilterInfo*>(*data);
				(*pFilterInfo).pPropertyService		= pPropertyService;
				(*pFilterInfo).radius				= 0.0;

				bool	restart		= true;
				double	radius		= 0.0;
				
				TriglavPlugInInt	blockIndex	= 0;
				while( true )
				{
					if( restart )
					{
						restart	= false;

						TriglavPlugInInt	processResult;
						TriglavPlugInFilterRunProcess(pRecordSuite,&processResult,(*pluginServer).hostObject,kTriglavPlugInFilterRunProcessStateStart);
						if( processResult == kTriglavPlugInFilterRunProcessResultExit ){ break; }

						if( (*pFilterInfo).radius > 0.0 )
						{
							blockIndex			= 0;

							radius			= (*pFilterInfo).radius;

							std::stringstream ss;
							ss << "Filter Start. Radius: " << radius << ", BlockCount: " << blockRectCount;
							WriteLog(ss.str(), INFO);
						}				
						else
						{
							//	何もしない
							blockIndex	= blockRectCount;
							TriglavPlugInFilterRunUpdateDestinationOffscreenRect(pRecordSuite,(*pluginServer).hostObject,&selectAreaRect);
						}
					}
					
					if( blockIndex < blockRectCount )
					{
						TriglavPlugInFilterRunSetProgressDone(pRecordSuite,(*pluginServer).hostObject,blockIndex);

						TriglavPlugInRect	blockRect	= blockRects[blockIndex];
						TriglavPlugInPoint	dstPos;
						dstPos.x	= blockRect.left;
						dstPos.y	= blockRect.top;
						TriglavPlugInRect	dstTempRect = blockRect; // Initialize with blockRect

						TriglavPlugInPtr	dstImageAddress;
						TriglavPlugInInt	dstImageRowBytes;
						TriglavPlugInInt	dstImagePixelBytes;
						(*pOffscreenService).getBlockImageProc(&dstImageAddress,&dstImageRowBytes,&dstImagePixelBytes,&dstTempRect,destinationOffscreenObject,&dstPos);

						TriglavPlugInPtr	dstAlphaAddress;
						TriglavPlugInInt	dstAlphaRowBytes;
						TriglavPlugInInt	dstAlphaPixelBytes;
						(*pOffscreenService).getBlockAlphaProc(&dstAlphaAddress,&dstAlphaRowBytes,&dstAlphaPixelBytes,&dstTempRect,destinationOffscreenObject,&dstPos);
						
						TriglavPlugInPtr	selectAddress = NULL;
						TriglavPlugInInt	selectRowBytes = 0;
						TriglavPlugInInt	selectPixelBytes = 0;
						(*pOffscreenService).getBlockSelectAreaProc(&selectAddress,&selectRowBytes,&selectPixelBytes,&dstTempRect,selectAreaOffscreenObject,&dstPos);

						if( dstImageAddress != NULL && radius > 0.0 )
						{
							int k = (int)std::ceil(radius * 3.0);
							
							TriglavPlugInRect extentRect;
							(*pOffscreenService).getExtentRectProc(&extentRect, sourceOffscreenObject);
							
							// Use dstTempRect instead of blockRect for processing bounds
							TriglavPlugInRect requestRect = dstTempRect;
							requestRect.left -= k;
							requestRect.top -= k;
							requestRect.right += k;
							requestRect.bottom += k;
							
							TriglavPlugInRect validRect;
							validRect.left = std::max<int>(requestRect.left, extentRect.left);
							validRect.top = std::max<int>(requestRect.top, extentRect.top);
							validRect.right = std::min<int>(requestRect.right, extentRect.right);
							validRect.bottom = std::min<int>(requestRect.bottom, extentRect.bottom);

							std::stringstream ss;
							ss << "Block[" << blockIndex << "] ValidRect: (" << validRect.left << "," << validRect.top << ")-(" << validRect.right << "," << validRect.bottom << ")";
							// WriteLog(ss.str(), INFO);
							
							if (validRect.right > validRect.left && validRect.bottom > validRect.top) {
								TriglavPlugInBitmapObject srcBitmap = NULL;
								// Depth is bytes-per-pixel (e.g. 4 for RGBA), not bits-per-pixel.
								(*pBitmapService).createProc(&srcBitmap, validRect.right - validRect.left, validRect.bottom - validRect.top, dstImagePixelBytes, kTriglavPlugInBitmapScanlineHorizontalLeftTop);

								TriglavPlugInBitmapObject srcAlphaBitmap = NULL;
								// Depth is bytes-per-pixel (e.g. 1 for Alpha).
								(*pBitmapService).createProc(&srcAlphaBitmap, validRect.right - validRect.left, validRect.bottom - validRect.top, dstAlphaPixelBytes, kTriglavPlugInBitmapScanlineHorizontalLeftTop);

								if (srcBitmap != NULL && srcAlphaBitmap != NULL) {
									TriglavPlugInPoint bitmapPos = {0, 0};
									TriglavPlugInPoint offscreenPos = {validRect.left, validRect.top};
									
									TriglavPlugInInt resultGetBitmap = (*pOffscreenService).getBitmapProc(srcBitmap, &bitmapPos, sourceOffscreenObject, &offscreenPos, validRect.right - validRect.left, validRect.bottom - validRect.top, kTriglavPlugInOffscreenCopyModeNormal);
									if(resultGetBitmap != kTriglavPlugInCallResultSuccess){
										WriteLog("getBitmapProc (Normal) Failed", ERROR);
									}
									TriglavPlugInInt resultGetAlpha = (*pOffscreenService).getBitmapProc(srcAlphaBitmap, &bitmapPos, sourceOffscreenObject, &offscreenPos, validRect.right - validRect.left, validRect.bottom - validRect.top, kTriglavPlugInOffscreenCopyModeAlpha);
									if(resultGetAlpha != kTriglavPlugInCallResultSuccess){
										WriteLog("getBitmapProc (Alpha) Failed", ERROR);
									}
									
									TriglavPlugInPtr srcBaseAddress;
									TriglavPlugInInt srcRowBytes;
									TriglavPlugInInt srcPixelBytes;
									(*pBitmapService).getAddressProc(&srcBaseAddress, srcBitmap, &bitmapPos);
									(*pBitmapService).getRowBytesProc(&srcRowBytes, srcBitmap);
									(*pBitmapService).getPixelBytesProc(&srcPixelBytes, srcBitmap);

									TriglavPlugInPtr srcAlphaBaseAddress;
									TriglavPlugInInt srcAlphaRowBytes;
									// Alpha map is 8 bit, so pixel bytes is likely 1. 
									(*pBitmapService).getAddressProc(&srcAlphaBaseAddress, srcAlphaBitmap, &bitmapPos);
									(*pBitmapService).getRowBytesProc(&srcAlphaRowBytes, srcAlphaBitmap);
									
									int offsetY = dstTempRect.top - dstPos.y;
									int offsetX = dstTempRect.left - dstPos.x;

									BYTE* pDstImageAddressY = static_cast<BYTE*>(dstImageAddress) + offsetY * dstImageRowBytes + offsetX * dstImagePixelBytes;
									BYTE* pDstAlphaAddressY = NULL;
									if(dstAlphaAddress) {
										pDstAlphaAddressY = static_cast<BYTE*>(dstAlphaAddress) + offsetY * dstAlphaRowBytes + offsetX * dstAlphaPixelBytes;
									}
									const BYTE* pSelectAddressY = NULL;
									if(selectAddress) {
										pSelectAddressY = static_cast<const BYTE*>(selectAddress) + offsetY * selectRowBytes + offsetX * selectPixelBytes;
									}

									int loopCount = 0;

									// Use dstTempRect for loops
									for( int y=dstTempRect.top; y<dstTempRect.bottom; ++y )
									{
										BYTE* pDstImageAddressX = pDstImageAddressY;
										BYTE* pDstAlphaAddressX = pDstAlphaAddressY;
										const BYTE* pSelectAddressX = pSelectAddressY;

										for( int x=dstTempRect.left; x<dstTempRect.right; ++x )
										{
											double currentRadius = radius;
											if (pSelectAddressX != NULL) {
												currentRadius = radius * (*pSelectAddressX) / 255.0;
											}

											double rSum = 0.0, gSum = 0.0, bSum = 0.0, aSum = 0.0;
											double weightSum = 0.0;
											
											if (currentRadius < 0.1) {
												// Radius too small, perform simple copy (equivalent to kernel with only center 1.0)
												int sampleY = y;
												int sampleX = x;
												
												if (sampleY < extentRect.top) sampleY = extentRect.top;
												if (sampleY >= extentRect.bottom) sampleY = extentRect.bottom - 1;
												if (sampleX < extentRect.left) sampleX = extentRect.left;
												if (sampleX >= extentRect.right) sampleX = extentRect.right - 1;
												
												int bx = sampleX - validRect.left;
												int by = sampleY - validRect.top;
												
												if (bx >= 0 && bx < (validRect.right - validRect.left) && by >= 0 && by < (validRect.bottom - validRect.top)) 
												{
													BYTE* pSrcPixel = static_cast<BYTE*>(srcBaseAddress) + by * srcRowBytes + bx * srcPixelBytes;
													BYTE* pSrcAlphaPixel = static_cast<BYTE*>(srcAlphaBaseAddress) + by * srcAlphaRowBytes + bx;
													
													double valA = static_cast<double>(*pSrcAlphaPixel);
													rSum = static_cast<double>(pSrcPixel[r]) * valA;
													gSum = static_cast<double>(pSrcPixel[g]) * valA;
													bSum = static_cast<double>(pSrcPixel[b]) * valA;
													aSum = valA;
													weightSum = 1.0;
												}
											} else {
												for( int ky = -k; ky <= k; ++ky )
												{
													for( int kx = -k; kx <= k; ++kx )
													{
														int sampleY = y + ky;
														int sampleX = x + kx;
														
														if (sampleY < extentRect.top) sampleY = extentRect.top;
														if (sampleY >= extentRect.bottom) sampleY = extentRect.bottom - 1;
														if (sampleX < extentRect.left) sampleX = extentRect.left;
														if (sampleX >= extentRect.right) sampleX = extentRect.right - 1;
														
														int bx = sampleX - validRect.left;
														int by = sampleY - validRect.top;
														
														if (bx >= 0 && bx < (validRect.right - validRect.left) && by >= 0 && by < (validRect.bottom - validRect.top)) 
														{
															BYTE* pSrcPixel = static_cast<BYTE*>(srcBaseAddress) + by * srcRowBytes + bx * srcPixelBytes;
															BYTE* pSrcAlphaPixel = static_cast<BYTE*>(srcAlphaBaseAddress) + by * srcAlphaRowBytes + bx; // Alpha is 8bit (1byte)

															double w = Gaussian(static_cast<double>(kx), static_cast<double>(ky), currentRadius);
															double valA = static_cast<double>(*pSrcAlphaPixel);
															
															// Weight color with both kernel weight and alpha value to prevent black halo
															rSum += static_cast<double>(pSrcPixel[r]) * valA * w;
															gSum += static_cast<double>(pSrcPixel[g]) * valA * w;
															bSum += static_cast<double>(pSrcPixel[b]) * valA * w;
															aSum += valA * w;
															weightSum += w;
														}
													}
												}
											}
											
											if (weightSum > 0.0) {
												// Normalize alpha first
												double outA = aSum / weightSum;
												
												if (outA > 0.0) {
													// Normalize color by dividing by total accumulated alpha weighting (which is aSum)
													// Formula: OutR = Sum(R*A*w) / Sum(A*w)
													pDstImageAddressX[r] = static_cast<BYTE>(std::min(255.0, std::max(0.0, rSum / aSum)));
													pDstImageAddressX[g] = static_cast<BYTE>(std::min(255.0, std::max(0.0, gSum / aSum)));
													pDstImageAddressX[b] = static_cast<BYTE>(std::min(255.0, std::max(0.0, bSum / aSum)));
												} else {
													// If alpha is zero, color is irrelevant (fully transparent), keep as black or 0 to be safe
													pDstImageAddressX[r] = 0;
													pDstImageAddressX[g] = 0;
													pDstImageAddressX[b] = 0;
												}
												
												if( pDstAlphaAddressX != NULL ) {
													*pDstAlphaAddressX = static_cast<BYTE>(std::min(255.0, std::max(0.0, outA)));
												}
											}

											pDstImageAddressX += dstImagePixelBytes;
											if( pDstAlphaAddressX != NULL ) {
												pDstAlphaAddressX += dstAlphaPixelBytes;
											}

											if( pSelectAddressX != NULL ) {
												pSelectAddressX += selectPixelBytes;
											}
											
											if (loopCount == 0 && blockIndex == 0) {
												std::stringstream ss_pixel;
												ss_pixel << "Pixel[0] Org(RGBA): ?,?,?,? => New(RGBA): " 
													<< (int)pDstImageAddressY[r] << "," << (int)pDstImageAddressY[g] << "," << (int)pDstImageAddressY[b] << "," << (pDstAlphaAddressY ? (int)*pDstAlphaAddressY : -1);
												WriteLog(ss_pixel.str(), INFO);
											}
											loopCount++;
										}
										pDstImageAddressY += dstImageRowBytes;
										if( pDstAlphaAddressY != NULL ) {
											pDstAlphaAddressY += dstAlphaRowBytes;
										}
										if( pSelectAddressY != NULL ) {
											pSelectAddressY += selectRowBytes;
										}
									}
									
									(*pBitmapService).releaseProc(srcBitmap);
									if(srcAlphaBitmap != NULL){
										(*pBitmapService).releaseProc(srcAlphaBitmap);
									}
								} else {
									WriteLog("Failed to create srcBitmap or srcAlphaBitmap", ERROR);
								}
							} else {
								WriteLog("Invalid Rect", WARNING);
							}
						} else {
							if (dstImageAddress == NULL) WriteLog("dstImageAddress is NULL", ERROR);
							if (radius <= 0.0) WriteLog("radius is <= 0.0", WARNING);
						}

						TriglavPlugInFilterRunUpdateDestinationOffscreenRect(pRecordSuite,(*pluginServer).hostObject,&blockRect);
						++blockIndex;
					}
					
					TriglavPlugInInt	processResult;
					if( blockIndex < blockRectCount )
					{
						TriglavPlugInFilterRunProcess(pRecordSuite,&processResult,(*pluginServer).hostObject,kTriglavPlugInFilterRunProcessStateContinue);
					}
					else
					{
						TriglavPlugInFilterRunSetProgressDone(pRecordSuite,(*pluginServer).hostObject,blockIndex);
						TriglavPlugInFilterRunProcess(pRecordSuite,&processResult,(*pluginServer).hostObject,kTriglavPlugInFilterRunProcessStateEnd);
					}
					if( processResult == kTriglavPlugInFilterRunProcessResultRestart )
					{
						restart	= true;
					}
					else if( processResult == kTriglavPlugInFilterRunProcessResultExit )
					{
						break;
					}
				}
				*result	= kTriglavPlugInCallResultSuccess;
			}
		}
	}
	return;
}

