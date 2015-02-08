/*****************************************************
    Comtech Support for Leadcore
    harry edited on 2011.11.26.
*****************************************************/
#include <linux/delay.h>
#include <linux/skbuff.h>
//#include <plat/devs.h>
//#include <plat/sdhci.h>
#include <linux/ctype.h>

#include <linux/types.h>
#include <linux/syscalls.h>
#include <linux/module.h>
#include <linux/init.h>

#define WLAN_STATIC_BUF	// this feature for using static buffer on wifi driver 
#ifdef WLAN_STATIC_BUF

#define PREALLOC_WLAN_SEC_NUM		4
#define PREALLOC_WLAN_BUF_NUM		160
#define PREALLOC_WLAN_SECTION_HEADER	24


#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_BUF_NUM * 1024)

#define WLAN_MAC_ADDR_FILE	"/sys/devices/platform/msm-dbg-info/wlan_mac_address"

//#define isxdigit(c)  ((__ismask(c)&(_D|_X)) != 0)
#define TOLOWER(x) ((x) | 0x20)

#define WLAN_SKB_BUF_NUM	17
#define MAC_LEN            6

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

#endif /* WLAN_STATIC_BUF */

#ifdef WLAN_STATIC_BUF

#define MOD_PARAM_PATHLEN	2048
char fw_path[MOD_PARAM_PATHLEN];

struct wifi_mem_prealloc {
	void *mem_ptr;
	unsigned long size;
};

static struct wifi_mem_prealloc wifi_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static char tmp_mac[18];
static unsigned char *gp_wifi_mac;


unsigned char *convert_mac2u8(char *src, unsigned char *dest);

int wifi_get_mac_addr(unsigned char *buf)
{
	bool invalid_addr = true;
	int i = 0;
	printk(KERN_INFO">> get mac addr: %02x:%02x:%02x:%02x:%02x:%02x\n", gp_wifi_mac[0], gp_wifi_mac[1],
															gp_wifi_mac[2],  gp_wifi_mac[3],
															gp_wifi_mac[4],  gp_wifi_mac[5]);
#if 1		//modified by zq 													
	for (i = 0; i < 6; i++) {
		if (gp_wifi_mac[i] != 0) {
			invalid_addr = false;
			break;
		}
	}

	if (invalid_addr) {
		struct file *fd;
		char mac_addr[17] = {0,};
		ssize_t result;
		mm_segment_t old_fs;

		fd = filp_open(WLAN_MAC_ADDR_FILE, O_RDWR, 0);
		if (IS_ERR(fd)) {
			printk(KERN_INFO"Can't get wifi mac address from %s\n", WLAN_MAC_ADDR_FILE);
			convert_mac2u8("00:ee:dd:cc:bb:aa", gp_wifi_mac);
			memcpy(buf, gp_wifi_mac, MAC_LEN);	
			return -1;
		}
		
		old_fs=get_fs();
		set_fs(get_ds());
		result = fd->f_op->read(fd, mac_addr, 17, &fd->f_pos);	
		if (result > 0) {
			printk(KERN_INFO"mac addr from nv:%s\n", mac_addr);	
			convert_mac2u8(mac_addr, gp_wifi_mac);		
			invalid_addr=true;
			for (i = 0; i < 6; i++) {
				if (gp_wifi_mac[i] != 0) {
					invalid_addr = false;
					break;
				}
			}
			if (invalid_addr) {
				gp_wifi_mac[0] = 0x00;
				gp_wifi_mac[1] = 0xee;
				gp_wifi_mac[2] = 0xdd;
				gp_wifi_mac[3] = 0xcc;
				gp_wifi_mac[4] = 0xbb;
				gp_wifi_mac[5] = 0xaa;
			}				
			memcpy(buf, gp_wifi_mac, MAC_LEN);	
		}
		
		set_fs(old_fs);
   		filp_close(fd,NULL);
	} else {		
  		memcpy(buf, gp_wifi_mac, MAC_LEN);
	}
#else
	memcpy(buf, gp_wifi_mac, MAC_LEN);
#endif
   	return 0;
}
EXPORT_SYMBOL(wifi_get_mac_addr);

unsigned char simple_strtou8(const char *cp, char **endp, unsigned int base)
{
   unsigned char result = 0;

   while (isxdigit(*cp)) {
      unsigned int value;

       value = isdigit(*cp) ? *cp - '0' : TOLOWER(*cp) - 'a' + 10;
       if (value >= base)
           break;
       result = result * base + value;
       cp++;
   }
   if (endp) {
       *endp = (char *)cp;
   }

   return result;
}

unsigned char *convert_mac2u8(char *src, unsigned char *dest)
{
   char *p = src;
   char *q;
   int i;

   for (i = 0; i < MAC_LEN; i++) {
       q = p;
       dest[i] = simple_strtou8(q, &p, 16);
       if (*p == ':') {
           p++;
       }
   }

   return NULL;
}

#if 1
static ssize_t brcm_wifi_mac_show(struct kobject *kobj, struct kobj_attribute *attr, 
                                   char *buf)
{
	return sprintf(buf, ">> get mac addr: %02x:%02x:%02x:%02x:%02x:%02x\n", gp_wifi_mac[0], gp_wifi_mac[1],
															gp_wifi_mac[2],  gp_wifi_mac[3],
															gp_wifi_mac[4],  gp_wifi_mac[5]);
}
#endif

static ssize_t brcm_wifi_mac_store(struct kobject *kobj, struct kobj_attribute *attr, 
                                   const char *buf, size_t n)
{
   strncpy(tmp_mac, buf, 17);
printk(KERN_INFO"brcm_wifi_mac_store:%s\n", tmp_mac);

#if 0
   if (strlen(tmp_mac) == 0) {
       convert_mac2u8("ff:ee:dd:cc:bb:aa", gp_wifi_mac);
       printk("set default mac addr ff:ee:dd:cc:bb:aa \n");
       return n;
   }
#else
   if (!strncmp(tmp_mac, "00:00:00:00:00:00", 17)) {
       convert_mac2u8("00:ee:dd:cc:bb:aa", gp_wifi_mac);
       printk("set default mac addr 00:ee:dd:cc:bb:aa \n");
       return n;
   }
#endif
#if 0
   printk("----------------------\n");
   for (i = 0; i < 17; i++) {
       printk("%c", tmp_mac[i]);
   }
   printk("\n----------------------\n");
#endif
   convert_mac2u8(tmp_mac, gp_wifi_mac);

   return n;
}

static char brcm_fw_path[64];

static ssize_t brcm_fw_type_show(struct kobject *kobj, struct kobj_attribute *attr, 
                                   char *buf)
{
	if (brcm_fw_path[0] == '\0') {
		if ((fw_path != NULL) && (fw_path[0] != '\0')) {
			strcpy(brcm_fw_path, fw_path);
		} else {
			strcpy(brcm_fw_path, "/system/etc/bcm/fw_4330b2.bin");
		}
	}

	return sprintf(buf, "%s\n", brcm_fw_path);
}

static struct kobj_attribute brcm_mac_attr = {
   .attr = {
       .name = __stringify(brcm_mac),
       .mode = S_IRUGO | S_IWUGO,
   },
   .store = &brcm_wifi_mac_store,
   .show = &brcm_wifi_mac_show,
};

static struct kobj_attribute brcm_fw_attr = {
   .attr = {
       .name = __stringify(fw_type),
       .mode = S_IRUGO | S_IWUGO,
   },
   .show = &brcm_fw_type_show,
};

static struct attribute *brcm_mac_attrs[] = {
   &brcm_mac_attr.attr,
   &brcm_fw_attr.attr,
   NULL,
};

static struct attribute_group brcm_mac_attr_group = {
   .attrs = brcm_mac_attrs,
};

struct kobject *brcm_kobj;

void *wlan_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}
EXPORT_SYMBOL(wlan_mem_prealloc);

#define DHD_SKB_HDRSIZE			336
#define DHD_SKB_1PAGE_BUFSIZE	((PAGE_SIZE*1)-DHD_SKB_HDRSIZE)
#define DHD_SKB_2PAGE_BUFSIZE	((PAGE_SIZE*2)-DHD_SKB_HDRSIZE)
#define DHD_SKB_4PAGE_BUFSIZE	((PAGE_SIZE*4)-DHD_SKB_HDRSIZE)

static int __init init_wifi_mem(void)
{
	int i;
	int j;
	struct kobject *mac_kobj;
	int ret;

   brcm_kobj = kobject_create_and_add("brcm", NULL);
   if (!brcm_kobj) {
       printk(KERN_INFO "%s : failed to create get wifi mac properties\n", __func__);
	   return -1;
   }
   
   mac_kobj = kobject_create_and_add("wifi_mac", brcm_kobj);
   if (!mac_kobj) {
       printk(KERN_INFO "%s : failed to create get wifi mac properties\n", __func__);
	   goto kobj_create_failed;
   }

   ret = sysfs_create_group(mac_kobj, &brcm_mac_attr_group);
   if (ret < 0) {
       printk(KERN_INFO "%s : failed to create get wifi mac properties\n", __func__);
	   goto kobj_create_failed;
   }

   gp_wifi_mac = kmalloc(MAC_LEN, GFP_KERNEL);
   if (!gp_wifi_mac) {
       printk(KERN_INFO "%s : failed to kmalloc wifi_mac_buffer.\n", __func__);
	   goto kmalloc_failed;
   }

   memset(gp_wifi_mac, 0, MAC_LEN);

	for (i = 0; i < 8; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_1PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (; i < 16; i++) {
		wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_2PAGE_BUFSIZE);
		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	wlan_static_skb[i] = dev_alloc_skb(DHD_SKB_4PAGE_BUFSIZE);
	if (!wlan_static_skb[i])
		goto err_skb_alloc;

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wifi_mem_array[i].mem_ptr =
				kmalloc(wifi_mem_array[i].size, GFP_KERNEL);

		if (!wifi_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	printk("%s: WIFI MEM Allocated\n", __FUNCTION__);
	return 0;

 err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

 err_skb_alloc:
 	kfree(gp_wifi_mac);

	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

kmalloc_failed:
	sysfs_remove_group(mac_kobj, &brcm_mac_attr_group);

kobj_create_failed:
 	kobject_del(mac_kobj);
	return -ENOMEM;
}

module_init(init_wifi_mem);
#endif	/* WLAN_STATIC_BUF */
