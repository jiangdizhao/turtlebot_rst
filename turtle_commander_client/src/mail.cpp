/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Christoph Rösmann
 * 
 * The content is mostly taken of from the cURL example page: 
 * http://curl.haxx.se/libcurl/c/smtp-tls.html
 * Copyright (C) 1998 - 2015, Daniel Stenberg, <daniel@haxx.se>, et al.
 *********************************************************************/

#include <turtle_commander/mail.h>

#include <stdio.h>
#include <iostream>
#include <string.h>
#include <vector>
#include <curl/curl.h>

namespace mail
{

struct upload_status {
  int lines_read;
  std::string turtlebot_name = "Turtlebot";
  std::string subject = "Turtlebot News";
  std::string body = "Nothing to say...";
  std::string sender_address;
  std::string receiver_address;
};
 

std::vector<std::string> createText(const std::string& sender_address, const std::string& receiver_address,
                                    const std::string& turtlebot_name, const std::string& subject, const std::string& body)
{
  std::vector<std::string> text =  {//"Date: Mon, 09 Dec 2015 18:23:00 +1000\r\n",
                                    "To: <" + receiver_address + ">\r\n",
                                    "From: <" + sender_address + ">(" + turtlebot_name + ")\r\n",
                                    //"Cc: " CC "(Another example User)\r\n",
                                    //"Message-ID: <dcd7cb36-11db-487a-9f3a-e652a9458efd@rfcpedant.example.org>\r\n", // TODO
                                    "Subject: " + subject + "\r\n",
                                    "\r\n", /* empty line to divide headers from body, see RFC5322 */ 
                                    body + "\r\n",
                                    "\r\n"};
  return text;
}

static size_t payload_source(void *ptr, size_t size, size_t nmemb, void *userp)
{
  struct upload_status *upload_ctx = (struct upload_status *)userp;
  const char *data = nullptr;
 
  if((size == 0) || (nmemb == 0) || ((size*nmemb) < 1)) {
    return 0;
  }
 
  std::vector<std::string> text = createText(upload_ctx->sender_address, upload_ctx->receiver_address,
                                             upload_ctx->turtlebot_name, upload_ctx->subject, upload_ctx->body);
  if (upload_ctx->lines_read<text.size())
    data = text[upload_ctx->lines_read].c_str();
  
  if(data) 
  {
    size_t len = strlen(data);
    memcpy(ptr, data, len);
    upload_ctx->lines_read++;
 
    return len;
  }
 
  return 0;
}




bool sendMail(const std::string& turtlebot_name, const std::string& subject, const std::string& body,
              const std::string& sender_address, const std::string& smtp_server, const std::string& login, 
              const std::string& password, const std::string& receiver_address)
{
    CURL *curl;
    CURLcode res = CURLE_OK;
    struct curl_slist *recipients = NULL;
    struct upload_status upload_ctx;
    
    if (sender_address.empty() || receiver_address.empty() || smtp_server.empty() || login.empty() || password.empty() )
    {
        fprintf(stderr, "Cannot send mail: mail settings not specified.");
        return false;
    }
    
    upload_ctx.lines_read = 0;
    upload_ctx.turtlebot_name = turtlebot_name;
    upload_ctx.subject = subject;
    upload_ctx.body = body;
    upload_ctx.sender_address = sender_address;
    upload_ctx.receiver_address = receiver_address;
    curl = curl_easy_init();
    if(curl) 
    {
        /* Set username and password */ 
        curl_easy_setopt(curl, CURLOPT_USERNAME, login.c_str());
        curl_easy_setopt(curl, CURLOPT_PASSWORD, password.c_str());
    
        /* This is the URL for your mailserver. Note the use of port 587 here,
        * instead of the normal SMTP port (25). Port 587 is commonly used for
        * secure mail submission (see RFC4403), but you should use whatever
        * matches your server configuration. */ 
        curl_easy_setopt(curl, CURLOPT_URL, smtp_server.c_str());
    
        /* In this example, we'll start with a plain text connection, and upgrade
        * to Transport Layer Security (TLS) using the STARTTLS command. Be careful
        * of using CURLUSESSL_TRY here, because if TLS upgrade fails, the transfer
        * will continue anyway - see the security discussion in the libcurl
        * tutorial for more details. */ 
        curl_easy_setopt(curl, CURLOPT_USE_SSL, (long)CURLUSESSL_ALL);
    
        /* If your server doesn't have a valid certificate, then you can disable
        * part of the Transport Layer Security protection by setting the
        * CURLOPT_SSL_VERIFYPEER and CURLOPT_SSL_VERIFYHOST options to 0 (false).
        *   curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
        *   curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
        * That is, in general, a bad idea. It is still better than sending your
        * authentication details in plain text though.
        * Instead, you should get the issuer certificate (or the host certificate
        * if the certificate is self-signed) and add it to the set of certificates
        * that are known to libcurl using CURLOPT_CAINFO and/or CURLOPT_CAPATH. See
        * docs/SSLCERTS for more information. */ 
        //curl_easy_setopt(curl, CURLOPT_CAINFO, "/path/to/certificate.pem");
    
        /* Note that this option isn't strictly required, omitting it will result in
        * libcurl sending the MAIL FROM command with empty sender data. All
        * autoresponses should have an empty reverse-path, and should be directed
        * to the address in the reverse-path which triggered them. Otherwise, they
        * could cause an endless loop. See RFC 5321 Section 4.5.5 for more details.
        */ 
        curl_easy_setopt(curl, CURLOPT_MAIL_FROM, std::string("<" + sender_address + ">").c_str());
    
        /* Add two recipients, in this particular case they correspond to the
        * To: and Cc: addressees in the header, but they could be any kind of
        * recipient. */ 
        recipients = curl_slist_append(recipients, std::string("<" + receiver_address + ">").c_str());
        //recipients = curl_slist_append(recipients, CC);
        curl_easy_setopt(curl, CURLOPT_MAIL_RCPT, recipients);
    
        /* We're using a callback function to specify the payload (the headers and
        * body of the message). You could just use the CURLOPT_READDATA option to
        * specify a FILE pointer to read from. */ 
        curl_easy_setopt(curl, CURLOPT_READFUNCTION, payload_source);
        curl_easy_setopt(curl, CURLOPT_READDATA, &upload_ctx);
        curl_easy_setopt(curl, CURLOPT_UPLOAD, 1L);
    
        /* Since the traffic will be encrypted, it is very useful to turn on debug
        * information within libcurl to see what is happening during the transfer.
        */ 
        curl_easy_setopt(curl, CURLOPT_VERBOSE, 0); // 1L
    
        /* Send the message */ 
        res = curl_easy_perform(curl);
    
        /* Check for errors */ 
        if(res != CURLE_OK)
        fprintf(stderr, "Cannot send mail: curl_easy_perform() failed: %s\n",
                curl_easy_strerror(res));
    
        /* Free the list of recipients */ 
        curl_slist_free_all(recipients);
    
        /* Always cleanup */ 
        curl_easy_cleanup(curl);
    }
    
    return (int)res;
}



bool sendTelegram(const std::string& text, const std::string& chat_id, const std::string& token)
{
	CURL *curl;
	CURLcode res;
	
	/* In windows, this will init the winsock stuff */ 
	curl_global_init(CURL_GLOBAL_ALL);
	
	/* get a curl handle */ 
	curl = curl_easy_init();
	if(curl) 
	{
		/* First set the URL that is about to receive our POST. This URL can
		just as well be a https:// URL if that is what should receive the
		data. */ 
		//curl_easy_setopt(curl, CURLOPT_URL, std::string("https://api.telegram.org/bot" + token + "/sendMessage?chat_id=" + message_id + "&text=" + text).c_str());
		curl_easy_setopt(curl, CURLOPT_URL, std::string("https://api.telegram.org/bot" + token + "/sendMessage").c_str());
		/* Now specify the POST data */
		//std::string data = "chat_id=141053380 "; //+ std::string(curl_easy_escape(curl, "text=test", 9));
		//curl_easy_setopt(curl, CURLOPT_POSTFIELDS, data.c_str());

		char* encoded_text = curl_easy_escape(curl, text.c_str(), 0);
		std::string arguments = "text=" + std::string(encoded_text) + "&chat_id=" + chat_id;
		curl_easy_setopt(curl, CURLOPT_POSTFIELDS, arguments.c_str());
		
		if (encoded_text)
			curl_free(encoded_text);
		
// 		curl_easy_setopt(curl, CURLO, "chat_id=141053380"); // --data-urlencode \"text=Some complex text \"");
	
		/* Perform the request, res will get the return code */ 
		res = curl_easy_perform(curl);
		/* Check for errors */ 
		if(res != CURLE_OK)
		fprintf(stderr, "curl_easy_perform() failed: %s\n",
				curl_easy_strerror(res));
	
		/* always cleanup */ 
		curl_easy_cleanup(curl);
	}
	curl_global_cleanup();
	return 0;
}


} // end namespace mail